#include "hogFeature.h"

void GetHogFeature(Eigen::MatrixXf& feature,
                   const std::vector<Eigen::Vector2f>& pixels,
                   const cv::Mat_<uchar>& image,
                   int cell_size,
                   VlHogVariant vlhog_variant,
                   int num_cells,
                   int num_bins)
{
    const int patch_width_half = num_cells * (cell_size / 2);
    
    cv::Mat hog_descriptors; // We'll get the dimensions later from vl_hog_get_*
    
    const int num_landmarks = pixels.size(); // modified
    for (int i = 0; i < num_landmarks; ++i) {
        int x = (int)pixels[i](0);
        int y = (int)pixels[i](1);
        
        cv::Mat roi_img;
        if (x - patch_width_half < 0 || y - patch_width_half < 0 || x + patch_width_half >= image.cols || y + patch_width_half >= image.rows) {
            // The feature extraction location is too far near a border. We extend the
            // image (add a black canvas) and then extract from this larger image.
            int borderLeft = (x - patch_width_half) < 0 ? std::abs(x - patch_width_half) : 0; // x and y are patch-centers
            int borderTop = (y - patch_width_half) < 0 ? std::abs(y - patch_width_half) : 0;
            int borderRight = (x + patch_width_half) >= image.cols ? std::abs(image.cols - (x + patch_width_half)) : 0;
            int borderBottom = (y + patch_width_half) >= image.rows ? std::abs(image.rows - (y + patch_width_half)) : 0;
            cv::Mat extendedImage = image.clone();
            cv::copyMakeBorder(extendedImage, extendedImage, borderTop, borderBottom, borderLeft, borderRight, cv::BORDER_CONSTANT, cv::Scalar(0));
            cv::Rect roi((x - patch_width_half) + borderLeft, (y - patch_width_half) + borderTop, patch_width_half * 2, patch_width_half * 2); // Rect: x y w h. x and y are top-left corner.
            roi_img = extendedImage(roi).clone(); // clone because we need a continuous memory block
        }
        else {
            cv::Rect roi(x - patch_width_half, y - patch_width_half, patch_width_half * 2, patch_width_half * 2); // x y w h. Rect: x and y are top-left corner. Our x and y are center. Convert.
            roi_img = image(roi).clone(); // clone because we need a continuous memory block
        }
        roi_img.convertTo(roi_img, CV_32FC1); // vl_hog_put_image expects a float* (values 0.0f-255.0f)
        VlHog* hog = vl_hog_new(vlhog_variant, num_bins, false); // transposed (=col-major) = false
        vl_hog_put_image(hog, (float*)roi_img.data, roi_img.cols, roi_img.rows, 1, cell_size); // (the '1' is numChannels)
        int ww = static_cast<int>(vl_hog_get_width(hog)); // assert ww == hh == numCells
        int hh = static_cast<int>(vl_hog_get_height(hog));
        int dd = static_cast<int>(vl_hog_get_dimension(hog)); // assert ww=hogDim1, hh=hogDim2, dd=hogDim3
        
        cv::Mat hogArray(1, ww*hh*dd, CV_32FC1); // safer & same result. Don't use C-style memory management.
        
        vl_hog_extract(hog, hogArray.ptr<float>(0));
        
        vl_hog_delete(hog);
        cv::Mat hogDescriptor(hh*ww*dd, 1, CV_32FC1);
        // Stack the third dimensions of the HOG descriptor of this patch one after each other in a column-vector:
        for (int j = 0; j < dd; ++j) {
            cv::Mat hogFeatures(hh, ww, CV_32FC1, hogArray.ptr<float>(0) + j*ww*hh);
            hogFeatures = hogFeatures.t();
            hogFeatures = hogFeatures.reshape(0, hh*ww); // make it to a column-vector
            cv::Mat currentDimSubMat = hogDescriptor.rowRange(j*ww*hh, j*ww*hh + ww*hh);
            hogFeatures.copyTo(currentDimSubMat);
        }
        hogDescriptor = hogDescriptor.t(); // now a row-vector
        hog_descriptors.push_back(hogDescriptor);
    }
    // concatenate all the descriptors for this sample vertically (into a row-vector):
    //hog_descriptors = hog_descriptors.reshape(0, hog_descriptors.cols * num_landmarks).t();
    
    feature = Eigen::Map<Eigen::MatrixXf>(hog_descriptors.ptr<float>(0),hog_descriptors.cols,hog_descriptors.rows);
}




#endif /* hogFeature_hpp */
