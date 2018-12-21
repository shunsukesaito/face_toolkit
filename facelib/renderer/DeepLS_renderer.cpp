#include "DeepLS_renderer.h"

#include <tinyexr.h>
#include <gl_utility/sh_utils.h>

#include <gflags/gflags.h>
DEFINE_double(dls_spec_scale, 0.26, "default specular scale for deepLS");
DEFINE_double(dsl_mesomap_size, 2048.0, "mesomap size");

void DeepLSRenderer::init(std::string data_dir, std::string shader_dir, FaceModelPtr model)
{
    programs_["main"] = GLProgram(shader_dir, "lightstage.vert", "deep_ls.frag", DrawMode::TRIANGLES);
    programs_["depth"] = GLProgram(shader_dir, "depthmap.vert", "depthmap.frag", DrawMode::TRIANGLES);
    programs_["plane"] = GLProgram(shader_dir, "full_texture_bgr.vert", "full_texture_bgr.frag", DrawMode::TRIANGLES);
    
    auto& prog_main = programs_["main"];
    auto& prog_depth = programs_["depth"];
    auto& prog_pl = programs_["plane"];
    
    param_.enable_mask = true;
    param_.spec_scale = FLAGS_dls_spec_scale;
    param_.mesomap_size = FLAGS_dsl_mesomap_size;
    
    param_.init(prog_main);
    prog_main.createTexture("u_sample_mask", data_dir + "deepls_mask.png");
    fb_ = Framebuffer::Create(1, 1, RT_NAMES::count); // will be resized based on frame size
    fb_depth_ = Framebuffer::Create(1, 1, 0);
    
    prog_pl.createUniform("u_alpha", DataType::FLOAT);
    
    Camera::initializeUniforms(prog_main, U_CAMERA_MVP | U_CAMERA_MV | U_CAMERA_SHADOW | U_CAMERA_WORLD | U_CAMERA_POS);
    Camera::initializeUniforms(prog_depth, U_CAMERA_MVP);
    
    mesh_.init(prog_main, AT_POSITION | AT_NORMAL | AT_UV | AT_TAN);
    mesh_.init(prog_depth, AT_POSITION);
    
    mesh_.update_uv(model->uvs_, model->tri_uv_);
    mesh_.update(prog_main, AT_UV);
    
    plane_.init(prog_pl,0.5);
    prog_pl.createTexture("u_texture", fb_->color(RT_NAMES::all), fb_->width(), fb_->height());
    
    const int order = 2;
    const int numSHBasis = (order + 1) * (order + 1);

    std::vector<std::string> env_list;
    {
        std::ifstream fin(data_dir + "LS/env_list.txt");
        if(fin.is_open()){
            while(!fin.eof())
            {
                std::string tmp;
                fin >> tmp;
                env_list.push_back(tmp);
            }
        }
        else{
            std::cout << "Error: failed to open " << data_dir + "LS/env_list.txt" << std::endl;
        }
    }
    
    spec_env1_locations_.clear();
    spec_env2_locations_.clear();
    diff_env_locations_.clear();
    int diffEnv_w = 0;
    int diffEnv_h = 0;
    int specEnv_w = 0;
    int specEnv_h = 0;
    for(int i = 0; i < env_list.size(); ++i)
    {
        std::cout << "Loading... " << env_list[i];
        const char* err;
        /*
         **Apply diffuse convolution to the map for lambertian rendering
         */
        std::string filename = data_dir + "LS/env/diff_" + env_list[i] + ".exr";
        TinyExrImage diffuseEnv;
        int ret = LoadEXR(&diffuseEnv.buf, &diffuseEnv.width, &diffuseEnv.height, filename.c_str(), &err);
        if (ret != 0)
        {
            std::cout << "Error: diffEnv file isn't loaded correctly... " << filename << " " << err << std::endl;
            throw std::runtime_error("Error: diffEnv file isn't loaded correctly...");
        }
        diffuseEnv.FlipVertical();

        std::cout << " diffuse done...";

        filename = data_dir + "LS/env/ward0.15_" + env_list[i] + ".hdr.exr";
        TinyExrImage specularEnv1;
        ret = LoadEXR(&specularEnv1.buf, &specularEnv1.width, &specularEnv1.height, filename.c_str(), &err);
        if (ret != 0)
        {
            std::cout << "Error: diffEnv file isn't loaded correctly... " << filename << " " << err << std::endl;
            throw std::runtime_error("Error: diffEnv file isn't loaded correctly...");
        }
        specularEnv1.FlipVertical();
        
        filename = data_dir + "LS/env/ward0.37_" + env_list[i] + ".hdr.exr";
        TinyExrImage specularEnv2;
        ret = LoadEXR(&specularEnv2.buf, &specularEnv2.width, &specularEnv2.height, filename.c_str(), &err);
        if (ret != 0)
        {
            std::cout << "Error: diffEnv file isn't loaded correctly... " << filename << " " << err << std::endl;
            throw std::runtime_error("Error: diffEnv file isn't loaded correctly...");
        }
        specularEnv2.FlipVertical();
        std::cout << " specular done..." << std::endl;
        
        if(diffEnv_w == 0 || diffEnv_h == 0){
            diffEnv_w = diffuseEnv.width;
            diffEnv_h = diffuseEnv.height;
        }
        if(specEnv_w == 0 || specEnv_h == 0){
            specEnv_w = specularEnv1.width;
            specEnv_h = specularEnv1.height;
        }
        
        GLuint diff_env_location = GLTexture::CreateTexture(diffuseEnv);
        GLuint spec_env1_location = GLTexture::CreateTexture(specularEnv1);
        GLuint spec_env2_location = GLTexture::CreateTexture(specularEnv2);
        diff_env_locations_.push_back(diff_env_location);
        spec_env1_locations_.push_back(spec_env1_location);
        spec_env2_locations_.push_back(spec_env2_location);
    }
    param_.env_size = diff_env_locations_.size();

    prog_main.createTexture("u_sample_depth", fb_depth_->depth(), fb_depth_->width(), fb_depth_->height());
    prog_main.createTexture("u_sample_diff_env", diff_env_locations_[0], diffEnv_w, diffEnv_h);
    prog_main.createTexture("u_sample_spec_env1", spec_env1_locations_[0], specEnv_w, specEnv_h);
    prog_main.createTexture("u_sample_spec_env2", spec_env2_locations_[0], specEnv_w, specEnv_h);
    
    prog_main.createTexture("u_sample_diff_albedo", 0, 0, 0);
    prog_main.createTexture("u_sample_spec_albedo", 0, 0, 0);
    prog_main.createTexture("u_sample_disp", 0, 0, 0);
}

void DeepLSRenderer::render(const Camera& camera, const FaceData& fd)
{
    if(!show_)
        return;
    
    GLFWwindow* window = glfwGetCurrentContext();
    int w = camera.width_;
    int h = camera.height_;

    if((param_.sub_samp*w != fb_->width()) || (param_.sub_samp*h != fb_->height()))
        fb_->Resize(param_.sub_samp*w, param_.sub_samp*h, RT_NAMES::count);
    if((param_.sub_samp*w != fb_depth_->width()) || (param_.sub_samp*h != fb_depth_->height()))
        fb_depth_->Resize(param_.sub_samp*w, param_.sub_samp*h, 0);

    // render parameters update
    auto& prog_main = programs_["main"];
    auto& prog_pl = programs_["plane"];
    auto& prog_depth = programs_["depth"];
    
    param_.update(prog_main);
    
    prog_main.updateTexture("u_sample_diff_env", diff_env_locations_[param_.env_id]);
    prog_main.updateTexture("u_sample_spec_env1", spec_env1_locations_[param_.env_id]);
    prog_main.updateTexture("u_sample_spec_env2", spec_env2_locations_[param_.env_id]);
    
    const auto& maps = fd.maps();
    assert(maps.size() != 0);
    prog_main.updateTexture("u_sample_disp", (GLuint)maps[0]);
    prog_main.updateTexture("u_sample_diff_albedo", (GLuint)maps[1]);
    prog_main.updateTexture("u_sample_spec_albedo", (GLuint)maps[2]);
    
    prog_pl.setUniformData("u_alpha", param_.alpha);

    // camera parameters update
    camera.updateUniforms(prog_main, fd.RT(), U_CAMERA_MVP | U_CAMERA_MV | U_CAMERA_SHADOW | U_CAMERA_WORLD | U_CAMERA_POS);
    camera.updateUniforms(prog_depth, fd.RT(), U_CAMERA_MVP);
    
    // update mesh attributes
    Eigen::MatrixX3f tan, btan;
    calcTangent(tan, btan, fd.pts_, fd.uvs(), fd.tripts(), fd.triuv());

    mesh_.update_position(fd.pts_, fd.tripts());
    mesh_.update_tangent(tan, btan, fd.tripts());
    mesh_.update_normal(fd.nml_, fd.tripts());
    mesh_.update_uv(fd.uvs(), fd.triuv());

    mesh_.update(prog_main, AT_POSITION | AT_NORMAL | AT_UV | AT_TAN);
    mesh_.update(prog_depth, AT_POSITION);
    
    // NOTE: need to make sure the viewport size matches the framebuffer size
    fb_depth_->Bind();
    glViewport(0, 0, fb_depth_->width(), fb_depth_->height());
    clearBuffer(COLOR::COLOR_ALPHA);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glEnable(GL_CULL_FACE);
    prog_depth.draw();
    fb_depth_->Unbind();
    
    // draw mesh
    fb_->Bind();
    glViewport(0, 0, fb_->width(), fb_->height());
    clearBuffer(COLOR::COLOR_ALPHA);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glEnable(GL_CULL_FACE);
    prog_main.draw(wire_);

    fb_->Unbind();
    
    prog_pl.updateTexture("u_texture", fb_->color((uint)param_.location));
    glfwGetFramebufferSize(window, &w, &h);
    glViewport(0, 0, w, h);
    glDisable(GL_CULL_FACE);
    glEnable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    prog_pl.draw();
}

#ifdef FACE_TOOLKIT
void DeepLSRenderer::render(const FaceResult& result, int cam_id, int frame_id)
{
    render(result.cameras[cam_id], result.fd[frame_id]);
}
#endif

#ifdef WITH_IMGUI
void DeepLSRenderer::updateIMGUI()
{
    if (ImGui::CollapsingHeader(name_.c_str())){
        ImGui::Checkbox("show", &show_);
        ImGui::Checkbox("wire", &wire_);
        param_.updateIMGUI();
    }
}
#endif

RendererHandle DeepLSRenderer::Create(std::string name, bool show)
{
    auto renderer = new DeepLSRenderer(name, show);
    
    return RendererHandle(renderer);
}

