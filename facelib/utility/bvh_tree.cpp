/*
 MIT License
 
 Copyright (c) 2018 Shunsuke Saito
 
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 
 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.
 
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 */

#include "bvh_tree.h"

void BVHNode::init()
{
    isLeaf_ = false;
    left_ = right_ = nullptr;
    f1_ = f2_ = nullptr;
}

BVHNode::BVHNode(const Triangle& f)
{
    init();
    bbox_ = BBox(f);
    f1_ = std::make_shared<Triangle>(f);
    isLeaf_ = true;
}

BVHNode::BVHNode(const Triangle& f1, const Triangle& f2)
{
    init();
    BBox b1 = BBox(f1);
    BBox b2 = BBox(f2);
    bbox_ = BBox::merge(b1,b2);
    f1_ = std::make_shared<Triangle>(f1);
    f2_ = std::make_shared<Triangle>(f2);
    isLeaf_ = true;
}

bool BVHNode::hit(Ray& ray)
{
    if(!bbox_.rayIntersect(ray))
        return false;
    else
    {
        if(isLeaf_ == true)
        {
            if(Ray::rayTriangle(ray,*f1_))
                return true;
            
            if(f2_ == nullptr)
                return false;
            
            return (Ray::rayTriangle(ray,*f2_));
        }
        
        return ((left_->hit(ray))||(right_->hit(ray)));
    }
}

int BVHTree::split(int start, int size, float pivot, int axis)
{
    BBox btemp;
    float c;
    int idx = 0;
    
    for(int i = start; i < (start + size); ++i)
    {
        btemp = BBox(tri_[i]);
        c = btemp.c_[axis];
        
        if(c < pivot)
        {
            Triangle temp = tri_[i];
            tri_[i] = tri_[start + idx];
            tri_[start + idx] = temp;
            idx++;
        }
    }
    
    if((idx == 0) || (idx == size)) idx = size/2;
    return idx;
}

bool BVHTree::interTest(Ray &ray)
{
    ray.normalize();
    bool result = root_->hit(ray);
    return result;
}

BVHNodePtr BVHTree::buildBranch(int start, int size, int axis)
{
    if(size < 1)
    {
        std::cerr << "BVHTree::buildBranch - invalid size" << std::endl;
    }
    if(size == 1)
    {
        return std::make_shared<BVHNode>(tri_[start]);
    }
    if(size == 2)
    {
        return std::make_shared<BVHNode>(tri_[start],tri_[start+1]);
    }
    
    BBox btemp = BBox(tri_[start]);
    
    for(int i = start+1; i < start + size; ++i)
    {
        BBox btemp2(tri_[i]);
        btemp = BBox::merge(btemp,btemp2);
    }
    Eigen::Vector3f pivot = btemp.c_;
    
    int mid_point = split(start,size,pivot[axis],axis);
    
    BVHNodePtr result = std::make_shared<BVHNode>();
    
    result->bbox_ = btemp;
    result->left_ = buildBranch(start, mid_point, ((axis + 1)% 3));
    result->right_ = buildBranch(start + mid_point, size - mid_point, (axis + 1)%3);
    
    return result;
}

void BVHTree::build(MeshData* obj)
{
    std::cout << "Building BVH Tree..." << std::endl;
    
    const Eigen::VectorXf& pts = obj->pts();
    const Eigen::MatrixX3i& tri = obj->tripts();
    
    tri_.clear();
    
    for(int i = 0; i < tri.rows(); ++i)
    {
        tri_.push_back(Triangle(pts.b3(tri(i,0)),pts.b3(tri(i,1)),pts.b3(tri(i,2)),i));
    }
    
    root_ = std::make_shared<BVHNode>();
    
    if(tri_.size() == 1)
        root_ = std::make_shared<BVHNode>(tri_[0]);
    else if(tri_.size() == 2)
        root_ = std::make_shared<BVHNode>(tri_[0], tri_[1]);

    root_->bbox_ = BBox(tri_);
    
    Eigen::Vector3f pivot = root_->bbox_.c_;
    
    int mid_point = split(0,tri_.size(),pivot[0],0);
    root_->left_ = buildBranch(0,mid_point,1);
    root_->right_ = buildBranch(mid_point,tri_.size()-mid_point,1);
    
    std::cout <<"BVHTree Build Done..." << std::endl;
}

