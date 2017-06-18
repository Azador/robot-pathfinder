/*
 *
 */

#include <vector>
#include <cstdint>
#include <cmath>

namespace Pathfinder
{
  template<uint32_t size, class T=double>
  class Vector
  {
    public:
      Vector ();
      Vector (std::initializer_list<T> v);

      Vector<size, T> operator+ (const Vector<size, T> & other) const;
      Vector<size, T> operator- (const Vector<size, T> & other) const;
      Vector<size, T> operator* (T factor) const;
      Vector<size, T> operator/ (T divisor) const;
      T operator* (const Vector<size, T> & other) const;

      Vector<size, T> & operator+= (const Vector<size, T> & other);
      Vector<size, T> & operator-= (const Vector<size, T> & other);
      Vector<size, T> & operator*= (T factor);
      Vector<size, T> & operator/= (T divisor);

      T length  () const;
      T length2 () const;

      bool operator== (const Vector<size, T> & other) const;
      bool operator!= (const Vector<size, T> & other) const;

      T & operator[] (uint32_t index)             { return _v[index]; }
      const T & operator[] (uint32_t index) const { return _v[index]; }

    private:
      double _v[size];
  };

  template<uint32_t height, uint32_t width, class T=double>
  class Matrix
  {
    public:
      Matrix ();
      Matrix (std::initializer_list<T> rows);

      Vector<width, T> & operator[] (uint32_t row)             { return _rows[row]; }
      const Vector<width, T> & operator[] (uint32_t row) const { return _rows[row]; }

      Vector<height, T> operator* (const Vector<width, T> & v) const;

    private:
      Vector<width, T> _rows[height];
  };


  class Position : public Vector<2>
  {
    public:
      Position ();
      Position (const Vector<2> & v);
      Position (double x, double y);

      double & x ()             { return operator[] (0); }
      const double & x () const { return operator[] (0); }

      double & y ()             { return operator[] (1); }
      const double & y () const { return operator[] (1); }

      double distance (const Position & other) const;
  };

  class Line
  {
    public:
      Line (const Position & p1, const Position & p2);
      virtual ~Line ();

      virtual Position perpend (const Position & pos, double *t) const;
      virtual double distance (const Position & pos) const;
      virtual double distance (const Position & pos, double *t) const;

      const Position & getPosition1 () const;
      const Position & getPosition2 () const;
      Position getDirection () const;

    private:
      Position _p1;
      Position _p2;
  };

  class LineSegment : public Line
  {
    public:
      LineSegment (const Position & p1, const Position & p2);
      virtual ~LineSegment ();

      virtual Position perpend (const Position & pos, double *t) const;
  };

  template<uint32_t degree>
  class PolynomCurve
  {
    public:
      PolynomCurve ();

      Position get (double t) const;

    private:
      Vector<degree+1, Position> _coeff;
  };

  class Transformation : public Matrix<3, 3>
  {
    public:
      Transformation ();
      Transformation (const Position & translation, double rotation, double scale);

      void set (const Position & translation, double rotation, double scale);
      Position getTranslation () const;
      double getRotation () const;
      double getScale () const;

      Position transformPosition (const Position & pos) const;
      Position rotatePosition (const Position & pos) const;
  };



  //====================================================================
  //
  // Implementation of template classes
  //

  template<uint32_t size, class T>
  Vector<size, T>::Vector ()
  : _v ()
  {
  }

  template<uint32_t size, class T>
  Vector<size, T>::Vector (std::initializer_list<T> v)
  : _v ()
  {
    uint32_t i = 0;

    for (auto vi=v.begin (); vi != v.end () && i < size; ++vi, ++i)
      _v[i] = *vi;
  }

  template<uint32_t size, class T>
  Vector<size, T> Vector<size, T>::operator+ (const Vector<size, T> & other) const
  {
    Vector<size, T> result = *this;
    result += other;
    return result;
  }

  template<uint32_t size, class T>
  Vector<size, T> Vector<size, T>::operator- (const Vector<size, T> & other) const
  {
    Vector<size, T> result = *this;
    result -= other;
    return result;
  }

  template<uint32_t size, class T>
  Vector<size, T> Vector<size, T>::operator* (T factor) const
  {
    Vector<size, T> result = *this;
    result *= factor;
    return result;
  }

  template<uint32_t size, class T>
  Vector<size, T> Vector<size, T>::operator/ (T divisor) const
  {
    Vector<size, T> result = *this;
    result /= divisor;
    return result;
  }

  template<uint32_t size, class T>
  T Vector<size, T>::operator* (const Vector<size, T> & other) const
  {
    T result;
    for (uint32_t i=0; i < size; ++i)
      result += _v[i] * other._v[i];
    return result;
  }


  template<uint32_t size, class T>
  Vector<size, T> & Vector<size, T>::operator+= (const Vector<size, T> & other)
  {
    for (uint32_t i=0; i < size; ++i)
      _v[i] += other._v[i];
    return *this;
  }

  template<uint32_t size, class T>
  Vector<size, T> & Vector<size, T>::operator-= (const Vector<size, T> & other)
  {
    for (uint32_t i=0; i < size; ++i)
      _v[i] -= other._v[i];
    return *this;
  }

  template<uint32_t size, class T>
  Vector<size, T> & Vector<size, T>::operator*= (T factor)
  {
    for (uint32_t i=0; i < size; ++i)
      _v[i] *= factor;
    return *this;
  }

  template<uint32_t size, class T>
  Vector<size, T> & Vector<size, T>::operator/= (T divisor)
  {
    for (uint32_t i=0; i < size; ++i)
      _v[i] /= divisor;
    return *this;
  }

  template<uint32_t size, class T>
  T Vector<size, T>::length  () const
  {
    return sqrt (length2 ());
  }

  template<uint32_t size, class T>
  T Vector<size, T>::length2 () const
  {
    return *this * *this;
  }

  template<uint32_t size, class T>
  bool Vector<size, T>::operator== (const Vector<size, T> & other) const
  {
    for (uint32_t i=0; i < size; ++i)
      if (_v[i] != other._v[i])
	return false;
    return true;
  }

  template<uint32_t size, class T>
  bool Vector<size, T>::operator!= (const Vector<size, T> & other) const
  {
    for (uint32_t i=0; i < size; ++i)
      if (_v[i] != other._v[i])
  	return true;
    return false;
  }

  template<uint32_t height, uint32_t width, class T>
  Matrix<height, width, T>::Matrix ()
  : _rows ()
  {
  }

  template<uint32_t height, uint32_t width, class T>
  Matrix<height, width, T>::Matrix (std::initializer_list<T> rows)
  : _rows (rows)
  {
    uint32_t row = 0;

    for (auto rowi=rows.begin (); rowi != rows.end () && row < height; ++rowi, ++row)
      _rows[row] = *rowi;
  }

  template<uint32_t height, uint32_t width, class T>
  Vector<height, T> Matrix<height, width, T>::operator* (const Vector<width, T> & v) const
  {
    Vector<height, T> result;

    for (uint32_t row=0; row < height; ++ row)
      {
	result[row] = 0.0;
	for (uint32_t col=0; col < width; ++ col)
	  result[row] += _rows[row][col] * v[col];
      }

    return result;
  }

  template<uint32_t degree>
  PolynomCurve<degree>::PolynomCurve ()
  : _coeff ()
  {
  }

  template<uint32_t degree>
  Position PolynomCurve<degree>::get (double t) const
  {
    Position result (0, 0);
    for (uint32_t i=0; i <= degree; ++i)
      result += _coeff[i] * std::pow (t, i);

    return result;
  }
}
