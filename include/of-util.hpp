
#ifndef AK_OF_UTIL_HPP
#define AK_OF_UTIL_HPP

#include <Eigen/Dense>

#include <OpticalFlowTemplate.h>
#include <ReferenceOF.h>

namespace of_util
{

template <typename DerivedImg, typename DerivedKernel>
void conv_2d_sparse(
  Eigen::MatrixBase<DerivedImg>& result,
  const Eigen::MatrixBase<DerivedImg>& img,
  const Eigen::MatrixBase<DerivedKernel>& kernel
  // const std::vector<Eigen::Vector2i>& interest_points,
  // const int patch_size
)
{
  DerivedKernel block;

  double mask_value = NAN;

  uint r,s;
  /*
  // build a mask within the result matrix
  for (const auto& p : interest_points)
  {
    for (int x = p.x() - patch_size/2; x < p.x() + patch_size/2 + 1; x++)
    {
      for (int y = p.y() - patch_size/2; y < p.y() + patch_size/2 + 1; y++)
      {
        if (
          x >= 0 &&
          y >= 0 &&
          x < result.cols() &&
          y < result.rows()
        )
        {
          result(y, x) = mask_value;
        }
      }
    }
  }
  */
  for (r = kernel.rows(); r < result.rows() - kernel.rows(); r++)
  {
    for (s = kernel.cols(); s < result.cols() - kernel.cols(); s++)
    {
      if (result(r,s) == mask_value || true)
      {
        block = static_cast<DerivedKernel>(img.block(
          r,
          s,
          kernel.rows(),
          kernel.cols()
        ));

        result(r, s) = block.cwiseProduct(kernel).sum();
      }
    }
  }
}

template <typename DerivedResult, typename DerivedImg>
void scale(
  Eigen::MatrixBase<DerivedResult>& result,
  const Eigen::MatrixBase<DerivedImg>& img
)
{
  uint f = img.cols() / result.cols();

  for (int i = 0; i < result.rows(); i++)
  {
    for (int j = 0; j < result.cols(); j++)
    {
      result(i, j) = img.block(i*f, j*f, f, f).array().sum();
    }
  }
}

void calc_distances(
  std::vector<double>& dist,
  const std::vector<student::OFVector>& of_vectors,
  int image_width,
  double fov,
  double phi_dot,
  double v
)
{
  double a, f, disc;

  for (int i = 0; i < of_vectors.size(); i++)
  {
    a = - fov/2.0 * (of_vectors.at(i).pos.x()/image_width - 0.5);
    f = of_vectors.at(i).dir.x();

    if (std::abs(phi_dot) <= 0.1)
    {
      dist[i] = -std::sin(a) / f;
    }
    else 
    {
      disc = f/(2.0*phi_dot) + std::sin(a)*v / phi_dot;

      if (disc >= 0)
      {
        dist[i] = std::max(
          f / phi_dot + std::sqrt(disc),
          f / phi_dot - std::sqrt(disc)
        );
      }
      else 
      {
        dist[i] = NAN;
      }
    }
  }
}

class WindowSearch
{
public:

  double sad_max_;

  template <typename DerivedImg>
  void calc_optical_flow(
    std::vector<student::OFVector>& of_vectors,
    const Eigen::MatrixBase<DerivedImg>& img_new,
    const Eigen::MatrixBase<DerivedImg>& img_old,
    const std::vector<Eigen::Vector2i>& interest_points,
    int search_win,
    int corr_win
  )
  {
    Eigen::MatrixXf orig_patch(corr_win, corr_win);
    
    for (const auto& p : interest_points)
    {
      // top left corner of the patch
      int x = p.x() - corr_win / 2;
      int y = p.y() - corr_win / 2;

      if (
        x < 0 ||
        y < 0 ||
        x + corr_win > img_old.cols() ||
        y + corr_win > img_old.rows()
      )
        continue;

      orig_patch = img_old.block(y, x, corr_win, corr_win);

      int dx_min;
      int dy_min;
      float sad_min = std::numeric_limits<float>::infinity();

      for (int dx = -search_win/2; dx < search_win/2 + 1; dx++)
      {
        for (int dy = -search_win/2; dy < search_win/2 + 1; dy++)
        {
          if (
            x + dx >= 0 &&
            y + dy >= 0 &&
            x + dx + corr_win <= img_new.cols() &&
            y + dy + corr_win <= img_new.rows()
          )
          {
            double sad = (
              img_new.block(
                y + dy,
                x + dx,
                corr_win,
                corr_win
              ) - orig_patch
            ).array().abs().sum();

            if (sad < sad_min)
            {
              sad_min = sad;
              dx_min = dx;
              dy_min = dy;
            }
          }
        }
      }

      if (sad_min < sad_max_ * (1 + corr_win)*(1 + corr_win))
      {
        of_vectors.push_back(
          student::OFVector(
            p.x(),  // x position
            p.y(),  // y position
            dx_min,     // x direction
            dy_min     // y direction
          )
        );
      }
    }
  }
};

class LucasKanade
{
public:

  int patch_size_;

  Eigen::MatrixXf grad_x_;
  Eigen::MatrixXf grad_y_;

  // sobel kernels for x and y directions
  Eigen::Matrix3f sobel_x_, sobel_y_;


  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  
  LucasKanade()
  {
    sobel_x_ <<
      1,0,-1,
      2,0,-2,
      1,0,-1
    ;

    sobel_y_ = sobel_x_.transpose();
  }

  template <typename DerivedImg>
  void calc_optical_flow(
    std::vector<student::OFVector>& of_vectors,
    const Eigen::MatrixBase<DerivedImg>& img_new,
    const Eigen::MatrixBase<DerivedImg>& img_old,
    const std::vector<Eigen::Vector2i>& interest_points
  )
  {
    int patch_size = patch_size_;

    // ugly but quick way of checking whether gradient matrices have been initialized yet
    if (grad_x_.cols() != img_old.cols())
    {
      grad_x_.resize(img_old.rows(), img_old.cols());
      grad_y_.resize(img_old.rows(), img_old.cols());
    }

    // calculate the gradient in x and y direction using sobel
    conv_2d_sparse(
      grad_x_,
      img_old,
      sobel_x_
      // interestPoints,
      // patch_size
    );

    conv_2d_sparse(
      grad_y_,
      img_old,
      sobel_y_
      //interestPoints,
      //patch_size
    );

    Eigen::Vector2f dir;

    // patch will always have an uneven size due to x being in the center
    // so it may or may not be one px larger on each side than patch_size
    int num_equations = std::pow(2*(patch_size/2)+1, 2);
    
    Eigen::MatrixXf i_xy(num_equations, 2);
    Eigen::MatrixXf i_t(num_equations, 1);

    for (const auto& p : interest_points)
    {
      uint i = 0;
      for (int x = p.x() - patch_size/2; x < p.x() + patch_size/2 + 1; x++)
      {
        for (int y = p.y() - patch_size/2; y < p.y() + patch_size/2 + 1; y++)
        {
          i_xy(i, 0) = grad_x_(y, x);
          i_xy(i, 1) = grad_y_(y, x);

          i_t(i, 0) = -(img_old(y, x) - img_new(y, x));
          
          i++;
        }
      }

      dir = (i_xy.transpose() * i_xy).inverse() * i_xy.transpose() * i_t;

      if (std::isfinite(dir.norm()))
      {
        of_vectors.push_back(
          student::OFVector(
            p.x(),  // x position
            p.y(),  // y position
            dir.x(),
            dir.y()
          )
        );
      }
    }
  }
};

} // namespace of_util

#endif