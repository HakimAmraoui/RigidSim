#ifndef _QUATERNION_HPP_
#define _QUATERNION_HPP_
#include <string>

#include "Vector3.hpp"
#include "Matrix3x3.hpp"


class Quaternion {
public:
  tReal a;
  tReal b;
  tReal c;
  tReal d;
  Quaternion(
    tReal a, tReal b, tReal c, tReal d) :
    a(a), b(b), c(c), d(d) {}

  ~Quaternion(){}

  Quaternion operator*(Quaternion const &x) const{
    //https://lucidar.me/fr/quaternions/quaternion-product/
    return Quaternion(
      this->a*x.a - this->b*x.b - this->c*x.c - this->d*x.d,
      this->a*x.b + this->b*x.a + this->c*x.d - this->d*x.c,
      this->a*x.c - this->b*x.d + this->c*x.a + this->d*x.b,
      this->a*x.d + this->b*x.c - this->c*x.b + this->d*x.a
    );
  }

  Quaternion operator+(Quaternion const &x)const{
    return Quaternion(
      this->a+x.a,
      this->b+x.b,
      this->c+x.c,
      this->d+x.d
    );
  }


  Mat3f toRotationMatrix(){
    return Mat3f(
      1-2*(this->c*this->c + this->d*this->d),
      2*  (this->b*this->c - this->a*this->d),
      2*  (this->b*this->d + this->a*this->c),

      2*  (this->b*this->c + this->a*this->d),
      1-2*(this->b*this->b + this->d*this->d),
      2*  (this->c*this->d - this->a*this->b),

      2*  (this->b*this->d - this->a*this->c),
      2*  (this->c*this->d + this->a*this->b),
      1-2*(this->b*this->b - this->c*this->c));
  }
  Quaternion operator*(tReal const &p) const{
    return Quaternion(p*this->a, p*this->b, p*this->c, p*this->d);
  }
  std::string toString(){
    return "("+ 
    std::to_string(this->a) + ", " +
    std::to_string(this->b) + ", " +
    std::to_string(this->c) + ", " +
    std::to_string(this->d) + ") ";
  }

  float dot(Quaternion &q) const{
    return this->a*q.a + this->b*q.b + this->c*q.c + this->d*q.d;
  }

  void normalize(){
    *this = *this * (1.0/std::sqrt(this->dot(*this)));
  }
};

#endif  /* _RIGIDSOLVER_HPP_ */
