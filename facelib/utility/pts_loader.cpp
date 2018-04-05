#include "pts_loader.h"

std::vector<Eigen::Vector3f> load_pts(std::string file)
{
    std::vector<Eigen::Vector3f> shape;
    std::ifstream fin;
    std::string temp;
    
    fin.open(file);
    if(!fin.is_open()){
        std::cout << "Warning: failed parsing pts from " << file << std::endl;
        return std::vector<Eigen::Vector3f>();
    }
    
    std::getline(fin, temp);
    std::getline(fin, temp, ' ');
    std::getline(fin, temp, ' ');
    std::getline(fin, temp);
    int n_land = std::stoi(temp);
    std::getline(fin, temp);
    for(int i = 0; i < n_land; ++i)
    {
        float x, y;
        fin >> x >> y;
        shape.push_back(Eigen::Vector3f(x,y,1.0));
    }
    fin.close();
    
    return shape;
}

bool write_pts(std::string file, const std::vector<Eigen::Vector3f>& pts)
{
    std::ofstream fout(file);
    if(!fout.is_open()){
        std::cout << "Warning: failed writing pts to " << file << std::endl;
        return false;
    }
    
    fout << "version: 1" << std::endl;
    fout << "n_points:  " << pts.size() << std::endl;
    fout << "{" << std::endl;
    for(int i = 0; i < pts.size(); ++i)
    {
        fout << pts[i][0] << " " << pts[i][1] << std::endl;
    }
    fout << "}";
    fout.close();
    
    return true;
}
