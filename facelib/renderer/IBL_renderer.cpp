#include "IBL_renderer.h"

#include <tinyexr.h>
#include <gl_utility/sh_utils.h>

void IBLRenderParams::init(GLProgram& prog)
{
    prog.createUniform("u_texture_mode", DataType::UINT);
    prog.createUniform("u_enable_mask", DataType::UINT);
    prog.createUniform("u_cull_occlusion", DataType::UINT);
    
    prog.createUniform("u_cull_offset", DataType::FLOAT);
    prog.createUniform("u_light_rot", DataType::FLOAT);
    
    prog.createUniform("u_uv_view", DataType::UINT);
}

void IBLRenderParams::update(GLProgram& prog)
{
    prog.setUniformData("u_texture_mode", (uint)texture_mode);
    prog.setUniformData("u_enable_mask", (uint)enable_mask);
    prog.setUniformData("u_cull_occlusion", (uint)enable_cull_occlusion);
    
    prog.setUniformData("u_cull_offset", cull_offset);
    prog.setUniformData("u_light_rot", light_rot);
    
    prog.setUniformData("u_uv_view", (uint)uv_view);
}

#ifdef WITH_IMGUI
void IBLRenderParams::updateIMGUI()
{
    const char* listbox_items1[] = { "None", "UV", "Image"};
    ImGui::ListBox("TextureMode", &texture_mode, listbox_items1, 3);
    
    ImGui::Checkbox("mask", &enable_mask);
    ImGui::Checkbox("uv view", &uv_view);
    ImGui::Checkbox("cull occlusion", &enable_cull_occlusion);
    ImGui::SliderFloat("Transparency", &alpha, 0.0, 1.0);
    ImGui::SliderFloat("cullOffset", &cull_offset, -1.0, 0.0);
    ImGui::SliderFloat("light rot", &light_rot, -3.14, 3.14);
    ImGui::SliderInt("env ID", &env_id, 0, env_size-1);
}
#endif

void IBLRenderer::init(std::string data_dir, std::string shader_dir, FaceModelPtr model)
{
    programs_["IBL"] = GLProgram(shader_dir, "IBL.vert", "IBL.frag", DrawMode::TRIANGLES);
    programs_["depth"] = GLProgram(shader_dir, "depthmap.vert", "depthmap.frag", DrawMode::TRIANGLES);
    programs_["plane"] = GLProgram(shader_dir, "full_texture_bgr.vert", "full_texture_bgr.frag", DrawMode::TRIANGLES);
    
    auto& prog_IBL = programs_["IBL"];
    auto& prog_depth = programs_["depth"];
    auto& prog_pl = programs_["plane"];
    
    param_.init(prog_IBL);
    prog_IBL.createTexture("u_sample_mask", data_dir + "f2f_mask.png");
    
    prog_pl.createUniform("u_alpha", DataType::FLOAT);
    
    fb_depth_ = Framebuffer::Create(1, 1, 0);
    
    Camera::initializeUniforms(prog_IBL, U_CAMERA_MVP | U_CAMERA_MV | U_CAMERA_SHADOW | U_CAMERA_WORLD | U_CAMERA_POS);
    Camera::initializeUniforms(prog_depth, U_CAMERA_MVP);
    
    ball_.generateSphere(10.0,24,48,false);
    
    mesh_.init(prog_IBL, AT_POSITION | AT_NORMAL | AT_COLOR | AT_UV);
    mesh_.init(prog_depth, AT_POSITION);
    
    mesh_.update_uv(model->uvs_, model->tri_uv_);
    mesh_.update(prog_IBL, AT_UV);
    
    plane_.init(prog_pl,0.5);
    prog_pl.createTexture("u_texture", 1, fb_->width(), fb_->height());
    
    const int order = 2;
    const int numSHBasis = (order + 1) * (order + 1);
    const int diffHDRI_w = 256;
    const int diffHDRI_h = 128;
    TinyExrImage shBases[numSHBasis];
    {
        //load up to the third order
        //[NOTE]: RGBA
        //[NOTE]: assume the input SH bases images are 256x128.
        printf("Loading SH bases up to %d order.\n", order);
        //load all the SH bases and append it to a single image
        for (int l = 0; l <= order; l++)
        {
            for (int m = 0; m < 2 * l + 1; m++)
            {
                TinyExrImage sh;
                sh.AllocateWithClear(256, 128);
                
                CreateSphericalHarmonics(m-l, l, sh);
                sh.FlipVertical();
                shBases[l*l+m] = sh;
            }
        }
    }
    
    std::vector<std::string> sh_list;
    {
        std::ifstream fin(data_dir + "env_list.txt");
        if(fin.is_open()){
            while(!fin.eof())
            {
                std::string tmp;
                fin >> tmp;
                sh_list.push_back(tmp);
            }
        }
        else{
            std::cout << "Error: failed to open " << data_dir << "env_list.txt" << std::endl;
        }
    }
    
    spec_HDRI_locations_.clear();
    diff_HDRI_locations_.clear();
    int specHDRI_w = 0;
    int specHDRI_h = 0;
    for(int i = 0; i < sh_list.size(); ++i)
    {
        std::cout << "Loading... " << sh_list[i];
        Eigen::Matrix3Xf SHCoeff;
        ReadSHCoefficients(data_dir + "SH/" + sh_list[i] + "_coefficients.txt", order, SHCoeff);
        
        TinyExrImage sourceHDRImage;
        ReconstructSHfromSHImage(order, SHCoeff, shBases, sourceHDRImage);
        /*
         **Apply diffuse convolution to the map for lambertian rendering
         */
        TinyExrImage diffuseHDRI;
        PanoramaSphericalHarmonicsBlurFromSHImage(order, shBases, sourceHDRImage, diffuseHDRI);
        
        std::cout << " diffuse done...";

        std::string filename = data_dir + "specHDRI/ward0.37_" + sh_list[i] + ".exr";
        const char* err;
        printf("Loading: %s...", filename.c_str());
        TinyExrImage specularEnv;
        int ret = LoadEXR(&specularEnv.buf, &specularEnv.width, &specularEnv.height, filename.c_str(), &err);
        if (ret != 0)
        {
            std::cout << "Error: diffEnv file isn't loaded correctly... " << filename << " " << err << std::endl;
            throw std::runtime_error("Error: diffEnv file isn't loaded correctly...");
        }
        specularEnv.FlipVertical();

        if(specHDRI_w == 0 || specHDRI_h == 0){
            specHDRI_w = specularEnv.width;
            specHDRI_h = specularEnv.height;
        }
        
        std::cout << " specular done..." << std::endl;
        
        //correct the exposure so it will be avarage 0.5
        //float scale = sourceHDRImage.CorrectExposure(0.5);
        //apply the same scale to the spec HDRI
        //specularHDRI.Scale(scale, scale, scale, 1.0);
        GLuint diff_HDRI_location = GLTexture::CreateTexture(diffuseHDRI);
        GLuint spec_HDRI_location = GLTexture::CreateTexture(specularEnv);
        diff_HDRI_locations_.push_back(diff_HDRI_location);
        spec_HDRI_locations_.push_back(spec_HDRI_location);
    }
    param_.env_size = diff_HDRI_locations_.size();

    prog_IBL.createTexture("u_sample_depth", fb_depth_->depth(), fb_depth_->width(), fb_depth_->height());
    prog_IBL.createTexture("u_sample_diffHDRI", diff_HDRI_locations_[0], diffHDRI_w, diffHDRI_h);
    prog_IBL.createTexture("u_sample_specHDRI", spec_HDRI_locations_[0], specHDRI_w, specHDRI_h);
}

void IBLRenderer::render(const Camera& camera, const FaceData& fd, bool draw_sphere)
{
    GLFWwindow* window = glfwGetCurrentContext();
    int w = camera.width_;
    int h = camera.height_;

    if((param_.sub_samp*w != fb_->width()) || (param_.sub_samp*h != fb_->height()))
        fb_->Resize(param_.sub_samp*w, param_.sub_samp*h, 1);
    if((param_.sub_samp*w != fb_depth_->width()) || (param_.sub_samp*h != fb_depth_->height()))
        fb_depth_->Resize(param_.sub_samp*w, param_.sub_samp*h, 0);
    
    // render parameters update
    auto& prog_IBL = programs_["IBL"];
    auto& prog_pl = programs_["plane"];
    auto& prog_depth = programs_["depth"];
    
    param_.update(prog_IBL);
    
    prog_pl.setUniformData("u_alpha", param_.alpha);
    
    prog_IBL.updateTexture("u_sample_diffHDRI", diff_HDRI_locations_[param_.env_id]);
    prog_IBL.updateTexture("u_sample_specHDRI", spec_HDRI_locations_[param_.env_id]);
    
    // camera parameters update
    camera.updateUniforms(prog_IBL, fd.getRT(), U_CAMERA_MVP | U_CAMERA_MV | U_CAMERA_SHADOW | U_CAMERA_WORLD | U_CAMERA_POS);
    camera.updateUniforms(prog_depth, fd.getRT(), U_CAMERA_MVP);
    
    // update mesh attributes
    mesh_.update_position(fd.pts_, fd.tripts());
    mesh_.update_color(fd.clr_, fd.tripts());
    mesh_.update_normal(fd.nml_, fd.tripts());
    mesh_.update_uv(fd.uvs(), fd.triuv());

    mesh_.update(prog_IBL, AT_POSITION | AT_COLOR | AT_NORMAL | AT_UV);
    mesh_.update(prog_depth, AT_POSITION);
    
    // NOTE: need to make sure the viewport size matches the framebuffer size
    fb_depth_->Bind();
    glViewport(0, 0, fb_depth_->width(), fb_depth_->height());
    clearBuffer(COLOR::COLOR_ALPHA);
    prog_depth.draw();
    fb_depth_->Unbind();
    
    // draw mesh
    fb_->Bind();
    glViewport(0, 0, fb_->width(), fb_->height());
    clearBuffer(COLOR::COLOR_ALPHA);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glEnable(GL_CULL_FACE);
    prog_IBL.draw(wire_);
    fb_->Unbind();
    
    glfwGetFramebufferSize(window, &w, &h);
    glViewport(0, 0, w, h);
    glDisable(GL_CULL_FACE);
    glEnable(GL_BLEND);
    glDisable(GL_DEPTH_TEST);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    prog_pl.draw();
    
    if(draw_sphere){
        glEnable(GL_DEPTH_TEST);
        glDisable(GL_BLEND);
        glEnable(GL_CULL_FACE);
        int sp_w = (int)(0.2*(float)std::min(w,h));
        glViewport(w-sp_w, 0, sp_w, sp_w);
        camera.updateUniforms4Sphere(prog_IBL, U_CAMERA_MVP | U_CAMERA_MV | U_CAMERA_SHADOW | U_CAMERA_WORLD | U_CAMERA_POS);
        ball_.update(prog_IBL, AT_POSITION | AT_COLOR | AT_NORMAL | AT_UV);
        prog_IBL.draw();
        glViewport(0, 0, w, h);
    }
}

#ifdef FACE_TOOLKIT
void IBLRenderer::render(const FaceResult& result, int cam_id, int frame_id)
{
    if(show_)
        render(result.cameras[cam_id], result.fd[frame_id], show_sphere_);
}
#endif

#ifdef WITH_IMGUI
void IBLRenderer::updateIMGUI()
{
    if (ImGui::CollapsingHeader(name_.c_str())){
        ImGui::Checkbox("show", &show_);
        ImGui::Checkbox("wire", &wire_);
        ImGui::Checkbox("show sphere", &show_sphere_);
        param_.updateIMGUI();
    }
}
#endif

RendererHandle IBLRenderer::Create(std::string name, bool show)
{
    auto renderer = new IBLRenderer(name, show);
    
    return RendererHandle(renderer);
}


