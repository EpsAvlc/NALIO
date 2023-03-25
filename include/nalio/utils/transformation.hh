#ifndef UNOS_UTILS_TRANSFORM_HH
#define UNOS_UTILS_TRANSFORM_HH

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace nalio {
template <typename T>
struct Transformation {
  Eigen::Matrix<T, 3, 1> translation;
  Eigen::Quaternion<T> rotation;
  double timestamp = -1;

  Transformation() {
    static_cast<void>(rotation.setIdentity());
    static_cast<void>(translation.setZero());
  }

  Transformation(Eigen::Quaternion<T> const& rotation_input, Eigen::Matrix<T, 3, 1> const& translation_input,
                 double const& timestamp_input = 0)
      : translation(translation_input), rotation(rotation_input), timestamp(timestamp_input) {}

  template <typename U>
  Transformation(Transformation<U> const& other) {
    timestamp = other.timestamp;
    translation = other.translation.template cast<T>();
    rotation = other.rotation.template cast<T>();
  }

  void reset() {
    timestamp = -1;
    static_cast<void>(rotation.setIdentity());
    static_cast<void>(translation.setZero());
  }

  template <typename U>
  Transformation<U> cast() {
    Transformation<U> cast_trans;
    cast_trans.translation = translation.template cast<U>();
    cast_trans.rotation = rotation.template cast<U>();
    return cast_trans;
  }

  // get rpy with ZYX rotation
  Eigen::Matrix<T, 3, 1> getEularAngle() const {
    Eigen::Matrix<T, 3, 3> rotation_matrix{rotation.toRotationMatrix()};
    std::double_t r00{rotation_matrix(0, 0)};
    std::double_t r01{rotation_matrix(0, 1)};
    std::double_t r02{rotation_matrix(0, 2)};
    std::double_t r10{rotation_matrix(1, 0)};
    std::double_t r11{rotation_matrix(1, 1)};
    // std::double_t r12{rotation_matrix(1,2)};
    std::double_t r20{rotation_matrix(2, 0)};
    std::double_t r21{rotation_matrix(2, 1)};
    std::double_t r22{rotation_matrix(2, 2)};

    Eigen::Matrix<T, 3, 1> eular_angle;
    if (((r00 * r00) + (r11 * r11)) < 0) {
      std::cout << "Error! Violate the sqrt arithmic rule" << std::endl;
      return eular_angle;
    }
    std::double_t sy{std::sqrt((r00 * r00) + (r11 * r11))};
    bool singular{sy < 1e-6};
    if (!singular) {
      eular_angle[0] = std::atan2(r21, r22);
      eular_angle[1] = std::atan2(r20, sy);
      eular_angle[2] = std::atan2(r10, r00);
    } else {
      eular_angle[0] = std::atan2(r01, r02);
      eular_angle[1] = -std::atan2(r20, sy);
      eular_angle[2] = 0.0;
    }
    return eular_angle;
  }

  Eigen::Matrix<T, 4, 4> matrix() const {
    Eigen::Matrix<T, 4, 4> matrix_input;
    matrix_input.setIdentity();
    matrix_input.block(0, 0, 3, 3) = rotation.toRotationMatrix();
    matrix_input.block(0, 3, 3, 1) = translation;
    return matrix_input;
  }

  Transformation inverse() const {
    Transformation<T> mtrans_inv;

    mtrans_inv.rotation = this->rotation.conjugate();
    mtrans_inv.translation = -(this->rotation.conjugate() * this->translation);
    return mtrans_inv;
  }

  Transformation interpoly(std::double_t scale, Transformation<T> const& other) const {
    Transformation<T> interpoly_trans;
    std::double_t minus_scale{1.0 - scale};
    interpoly_trans.translation = this->translation * minus_scale + scale * other.translation;
    interpoly_trans.rotation = this->rotation.slerp(scale, other.rotation);
    interpoly_trans.timestamp = this->timestamp * minus_scale + scale * other.timestamp;
    return interpoly_trans;
  }

  Transformation operator*(Transformation const& other) const {
    Eigen::Matrix<T, 4, 4> mutiple_matrix = this->matrix() * other.matrix();
    Transformation<T> mtrans_multiple;

    Eigen::Matrix<T, 3, 3> rotation_matrix = mutiple_matrix.block(0, 0, 3, 3);
    Eigen::Quaternion<T> rotation(rotation_matrix);
    mtrans_multiple.rotation = rotation;
    mtrans_multiple.translation = mutiple_matrix.block(0, 3, 3, 1);
    return mtrans_multiple;
  }

  Eigen::Matrix<T, 3, 1> operator*(Eigen::Matrix<T, 3, 1> const& point) const {
    Eigen::Matrix<T, 3, 1> transformed_point{0, 0, 0};
    transformed_point = this->rotation.toRotationMatrix() * point + this->translation;
    return transformed_point;
  }

  friend std::ostream& operator<<(std::ostream& os, Transformation<T> const pose) {
    os << "[(" << pose.translation.transpose() << "),(" << pose.rotation.coeffs().transpose() << ")]";
    return os;
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using TransformationD = Transformation<double>;
using TransformationF = Transformation<float>;
}  // namespace nalio

#endif  // UNOS_UTILS_TRANSFORM_HH
