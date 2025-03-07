
#ifndef MM_BASIC_TENSOR_H
#define MM_BASIC_TENSOR_H

// System
#include <cmath>
#include <iostream>

// Local
#include <miemie/basic/types.h>
#include <miemie/basic/vec.h>

namespace mm
{
    /**
     * @brief  The tensor matrix - [3*3] second order symmetric non-negative definite matrix.
     *
     * The class is able to hold the eigenvalues and eigenvectors of the tensor matrix.
     *
     * This class provides a representation of a tensor with member attributes for tensor matrix values,
     * eigenvalues, and eigenvectors. It includes constructors, getters, setters, and various operator
     * overloads for tensor manipulation.
     *
     *
     * @tparam FT The type of the tensor matrix, must be floating point type.
     *
     * @note Turning vectors into tensors can easily be down by the copy constructor of the
     * tensor class copy constructor having a vector as argument. This means that all
     * operators can be used when working with both vectors and tensors.
     *
     * For instance, adding a 3D vector "Vec" of type Vec3f to a tensor "Tensor" of
     * type Tensor<float> can be down like this:
     *
     *  Tensor += Tensor3<float>(Vec);
     *
     */
    template <typename FT>
    class Tensor
    {
    public:
    protected:
        // upper left velue in tensor matrix (X^2)
        FT m_m200;
        // center value in tensor matrix (Y^2)
        FT m_m020;
        // lower right value in tensor matrix (Z^2)
        FT m_m002;

        // tensor matrix (XZ)
        FT m_m101;
        // tensor matrix (YZ)
        FT m_m011;
        // tensor matrix (XY)
        FT m_m110;

        // Eigen values - [in descending order];
        FT m_lambda1;
        FT m_lambda2;
        FT m_lambda3;

        // The three eigenvectors corresponding to the three eigenvalues;
        Vec<3, FT> m_e1;
        Vec<3, FT> m_e2;
        Vec<3, FT> m_e3;

    public:
        /**
         * @brief Construct a new Tensor object
         *
         */
        Tensor()
            : m_m200(FT(0)),
              m_m020(FT(0)),
              m_m002(FT(0)),
              m_m011(FT(0)),
              m_m101(FT(0)),
              m_m110(FT(0)),
              m_lambda1(FT(0)),
              m_lambda2(FT(0)),
              m_lambda3(FT(0)),
              m_e1(Vec<3, FT>()),
              m_e2(Vec<3, FT>()),
              m_e3(Vec<3, FT>())
        {
        }

        /**
         * @brief Construct a new Tensor object
         *
         * @param M200 The upper left value in the tensor matrix (X^2)
         * @param M020 The center value in the tensor matrix (Y^2)
         * @param M002 The lower right value in the tensor matrix (Z^2)
         * @param M110 The tensor matrix (YZ)
         * @param M101 The tensor matrix (XZ)
         * @param M011 The tensor matrix (XY)
         */
        Tensor(FT M200, FT M020, FT M002, FT M110, FT M101, FT M011)
            : m_m200(M200),
              m_m020(M020),
              m_m002(M002),
              m_m110(M110),
              m_m101(M101),
              m_m011(M011),
              m_lambda1(FT(0)),
              m_lambda2(FT(0)),
              m_lambda3(FT(0)),
              m_e1(Vec<3, FT>()),
              m_e2(Vec<3, FT>()),
              m_e3(Vec<3, FT>())
        {
        }

        /**
         * @brief Construct a new Tensor object
         *
         * This is constructor initializes a Tensor object by copying the values from the provided
         * Tensor object. It also initializes the eigenvalues and eigenvectors to the values of the
         * provided Tensor object.
         * @tparam T1 The type of the input tensor.
         * @param tensor The input tensor from which to initialize the Tensor object.
         */
        template <typename T1>
        inline Tensor(const Tensor<T1> &tensor)
            : m_m200(static_cast<FT>(tensor.get_m200())),
              m_m020(static_cast<FT>(tensor.get_m020())),
              m_m002(static_cast<FT>(tensor.get_m002())),
              m_m011(static_cast<FT>(tensor.get_m011())),
              m_m101(static_cast<FT>(tensor.get_m101())),
              m_m110(static_cast<FT>(tensor.get_m110())),
              m_lambda1(tensor.get_lambda1()),
              m_lambda2(tensor.get_lambda2()),
              m_lambda3(tensor.get_lambda3()),
              m_e1(tensor.get_e1()),
              m_e2(tensor.get_e2()),
              m_e3(tensor.get_e3())
        {
        }

        /**
         * @brief Construct a new Tensor object
         *
         * @tparam T1 The type of the input direction.
         * @param vec3 The input significant/dominant direction vector.
         */
        template <typename T1>
        inline Tensor(const Vec<3, T1> &vec3)
            : m_m200(static_cast<FT>(std::pow(vec3.get_x(), 2))),
              m_m020(static_cast<FT>(std::pow(vec3.get_y(), 2))),
              m_m002(static_cast<FT>(std::pow(vec3.get_z(), 2))),
              m_m110(static_cast<FT>(vec3.get_x() * vec3.get_y())),
              m_m101(static_cast<FT>(vec3.get_x() * vec3.get_z())),
              m_m011(static_cast<FT>(vec3.get_y() * vec3.get_z()))
        {
        }

        ~Tensor()
        {
        }

        /*  -------------			Getters and Setters					  -------------- */
    public:
        /**
         * @brief Set the tensor matrix values.
         *
         * @param M200 The upper left value in the tensor matrix (X^2)
         * @param M020 The center value in the tensor matrix (Y^2)
         * @param M002 The lower right value in the tensor matrix (Z^2)
         * @param M110 The tensor matrix (XY)
         * @param M101 The tensor matrix (XZ)
         * @param M011 The tensor matrix (YZ)
         */
        inline FT get_m200() const
        {
            return m_m200;
        }
        inline FT get_m020() const
        {
            return m_m020;
        }
        inline FT get_m002() const
        {
            return m_m002;
        }
        inline FT get_m110() const
        {
            return m_m110;
        }
        inline FT get_m101() const
        {
            return m_m101;
        }
        inline FT get_m011() const
        {
            return m_m011;
        }

        inline FT get_lambda1() const
        {
            return m_lambda1;
        }
        inline FT get_lambda2() const
        {
            return m_lambda2;
        }
        inline FT get_lambda3() const
        {
            return m_lambda3;
        }
        inline FT get_lambda_sum() const
        {
            return (m_lambda1 + m_lambda2 + m_lambda3);
        }
        inline Vec<3, FT> get_e1() const
        {
            return m_e1;
        }
        inline Vec<3, FT> get_e2() const
        {
            return m_e2;
        }
        inline Vec<3, FT> get_e3() const
        {
            return m_e3;
        }

        /**
         * @brief Set the tensor matrix values.
         *
         * @param M200 The upper left value in the tensor matrix (X^2)
         * @param M020 The center value in the tensor matrix (Y^2)
         * @param M002 The lower right value in the tensor matrix (Z^2)
         * @param M110 The tensor matrix (XY)
         * @param M101 The tensor matrix (XZ)
         * @param M011 The tensor matrix (YZ)
         */
        inline void set(const FT &M200, const FT &M020, const FT &M002,
                        const FT &M110, const FT &M101, const FT &M011)
        {
            this->m_m200 = M200;
            this->m_m020 = M020;
            this->m_m002 = M002;
            this->m_m110 = M110;
            this->m_m101 = M101;
            this->m_m011 = M011;
        }
        inline void set_m200(const FT &M200)
        {
            this->m_m200 = M200;
        }
        inline void set_m020(const FT &M020)
        {
            this->m_m020 = M020;
        }
        inline void set_m002(const FT &M002)
        {
            this->m_m002 = M002;
        }
        inline void set_m110(const FT &M110)
        {
            this->m_m110 = M110;
        }
        inline void set_m101(const FT &M101)
        {
            this->m_m101 = M101;
        }
        inline void set_m011(const FT &M011)
        {
            this->m_m011 = M011;
        }

        /**
         * @brief This function calculates the eigen system of the tensor.
         *
         * @tparam T1 The type of the input tensor.
         * @param vec3 The input tensor from which to initialize the Tensor object.
         */
        template <typename T1>
        inline void set(const Vec<3, T1> &vec3)
        {
            this->m_m200 = static_cast<FT>(std::pow(vec3.get_x(), 2));
            this->m_m020 = static_cast<FT>(std::pow(vec3.get_y(), 2));
            this->m_m002 = static_cast<FT>(std::pow(vec3.get_z(), 2));
            this->m_m110 = static_cast<FT>(vec3.get_x() * vec3.get_y());
            this->m_m101 = static_cast<FT>(vec3.get_x() * vec3.get_z());
            this->m_m011 = static_cast<FT>(vec3.get_y() * vec3.get_z());
        }

        inline void add(const FT &M200, const FT &M020, const FT &M002,
                        const FT &M110, const FT &M101, const FT &M011)
        {
            this->m_m200 += M200;
            this->m_m020 += M020;
            this->m_m002 += M002;
            this->m_m110 += M110;
            this->m_m101 += M101;
            this->m_m011 += M011;
        }
        inline void add_m200(const FT &M200)
        {
            this->m_m200 += M200;
        }
        inline void add_m020(const FT &M020)
        {
            this->m_m020 += M020;
        }
        inline void add_m002(const FT &M002)
        {
            this->m_m002 += M002;
        }
        inline void add_m110(const FT &M110)
        {
            this->m_m110 += M110;
        }
        inline void add_m101(const FT &M101)
        {
            this->m_m101 += M101;
        }
        inline void add_m011(const FT &M011)
        {
            this->m_m011 += M011;
        }

        inline void subtract(const FT &M200, const FT &M020, const FT &M002,
                             const FT &M110, const FT &M101, const FT &M011)
        {
            this->m_m200 -= M200;
            this->m_m020 -= M020;
            this->m_m002 -= M002;
            this->m_m110 -= M110;
            this->m_m101 -= M101;
            this->m_m011 -= M011;
        }
        inline void subtract_m200(const FT &M200)
        {
            this->m_m200 -= M200;
        }
        inline void subtract_m020(const FT &M020)
        {
            this->m_m020 -= M020;
        }
        inline void subtract_m002(const FT &M002)
        {
            this->m_m002 -= M002;
        }
        inline void subtract_m110(const FT &M110)
        {
            this->m_m110 -= M110;
        }
        inline void subtract_m101(const FT &M101)
        {
            this->m_m101 -= M101;
        }
        inline void subtract_m011(const FT &M011)
        {
            this->m_m011 -= M011;
        }

    public:
        /**
         * @brief Calculate the modulus (length) of the tensor.
         *
         * This function computes the modulus of the tensor, which is the square root
         * of the sum of the squares of the tensor matrix components m_m200, m_m020, and m_m002.
         * It provides a measure of the magnitude of the tensor.
         *
         * @return The modulus of the tensor.
         */
        inline FT get_modulus() const
        {
            return sqrt(m_m200 + m_m020 + m_m002);
        }

        /**
         * @brief This function copies the values of the provided tensor to the current tensor.
         *
         * @tparam T1 The type of the input tensor.
         * @param tensor The input tensor from which to copy the values.
         * @return Tensor<FT>& The current tensor object.
         */
        template <typename T1>
        inline Tensor<FT> &operator=(const Tensor<T1> &tensor)
        {
            if (this == reinterpret_cast<const Tensor<FT> *>(&tensor))
                return *this; // Avoid self-assignment

            this->m_m200 = static_cast<FT>(tensor.get_m200());
            this->m_m020 = static_cast<FT>(tensor.get_m020());
            this->m_m002 = static_cast<FT>(tensor.get_m002());
            this->m_m110 = static_cast<FT>(tensor.get_m110());
            this->m_m101 = static_cast<FT>(tensor.get_m101());
            this->m_m011 = static_cast<FT>(tensor.get_m011());

            return *this;
        }

        /**
         * @brief This function copies the values of the provided vector to the current tensor.
         *
         * @tparam T1 The type of the input vector.
         * @param vec3 The input vector from which to copy the values.
         * @return Tensor<FT>& The current tensor object.
         */
        template <typename T1>
        inline Tensor<FT> &operator=(const Vec<3, T1> &vec3)
        {
            this->m_m200 = static_cast<FT>(std::pow(vec3.get_x(), 2));
            this->m_m020 = static_cast<FT>(std::pow(vec3.get_y(), 2));
            this->m_m002 = static_cast<FT>(std::pow(vec3.get_z(), 2));
            this->m_m110 = static_cast<FT>(vec3.get_x() * vec3.get_y());
            this->m_m101 = static_cast<FT>(vec3.get_x() * vec3.get_z());
            this->m_m011 = static_cast<FT>(vec3.get_y() * vec3.get_z());

            // this->calc_eigen_system();

            return *this;
        }

        /**
         * @brief This function adds the values of the provided tensor to the current tensor.
         *
         * @tparam T1 The type of the input tensor.
         * @param tensor The input tensor to add to the current tensor.
         * @return Tensor<FT>&  The current tensor object.
         */
        template <typename T1>
        inline Tensor<FT> &operator+=(const Tensor<T1> &tensor)
        {
            this->m_m200 += static_cast<FT>(tensor.get_m200());
            this->m_m020 += static_cast<FT>(tensor.get_m020());
            this->m_m002 += static_cast<FT>(tensor.get_m002());
            this->m_m110 += static_cast<FT>(tensor.get_m110());
            this->m_m101 += static_cast<FT>(tensor.get_m101());
            this->m_m011 += static_cast<FT>(tensor.get_m011());

            // this->calc_eigen_system();

            return *this;
        }

        /**
         * @brief This function subtracts the values of the provided tensor from the current tensor.
         *
         * It subtracts the values of the provided tensor from the current tensor.
         *
         * @tparam T1 The type of the input tensor.
         * @param tensor The input tensor to subtract from the current tensor.
         * @return Tensor<FT>& The current tensor object.
         */
        template <typename T1>
        inline Tensor<FT> &operator-=(const Tensor<T1> &tensor)
        {
            this->m_m200 -= static_cast<FT>(tensor.get_m200());
            this->m_m020 -= static_cast<FT>(tensor.get_m020());
            this->m_m002 -= static_cast<FT>(tensor.get_m002());
            this->m_m110 -= static_cast<FT>(tensor.get_m110());
            this->m_m101 -= static_cast<FT>(tensor.get_m101());
            this->m_m011 -= static_cast<FT>(tensor.get_m011());

            // this->calc_eigen_system();
            return *this;
        }

        /**
         * @brief This function multiplies the current tensor by the provided scalar.
         *
         * It multiplies the current tensor by the provided scalar.
         *
         * @tparam T1 The type of the input scalar.
         * @param scalar The input scalar to multiply the current tensor by.
         * @return Tensor<FT>& The current tensor object.
         */
        template <typename T1>
        inline Tensor<FT> &operator*=(const T1 &scalar)
        {
            this->m_m200 *= static_cast<FT>(scalar);
            this->m_m020 *= static_cast<FT>(scalar);
            this->m_m002 *= static_cast<FT>(scalar);
            this->m_m110 *= static_cast<FT>(scalar);
            this->m_m101 *= static_cast<FT>(scalar);
            this->m_m011 *= static_cast<FT>(scalar);

            // this->calc_eigen_system();
            return *this;
        }

        /**
         * @brief This function divides the current tensor by the provided scalar.
         *
         * It divides the current tensor by the provided scalar.
         *
         * @tparam T1 The type of the input scalar.
         * @param scalar The input scalar to divide the current tensor by.
         * @return Tensor<FT>& The current tensor object.
         */
        template <typename T1>
        inline Tensor<FT> &operator/=(const T1 &scalar)
        {
            this->m_m200 /= static_cast<FT>(scalar);
            this->m_m020 /= static_cast<FT>(scalar);
            this->m_m002 /= static_cast<FT>(scalar);
            this->m_m110 /= static_cast<FT>(scalar);
            this->m_m101 /= static_cast<FT>(scalar);
            this->m_m011 /= static_cast<FT>(scalar);

            // this->calc_eigen_system();
            return *this;
        }

        /**
         * @brief This function adds the provided tensor to the current tensor.
         *
         * This function creates a new tensor object and adds the provided tensor to the current tensor.
         *
         * @tparam T1 The type of the input tensor.
         * @param tensor The input tensor to add to the current tensor.
         * @return Tensor<FT>&  The new tensor object.
         */
        template <typename T1>
        inline Tensor<FT> &operator+(const Tensor<T1> &tensor) const
        {
            return Tensor<FT>(
                this->m_m200 + static_cast<FT>(tensor.get_m200()),
                this->m_m020 + static_cast<FT>(tensor.get_m020()),
                this->m_m002 + static_cast<FT>(tensor.get_m002()),
                this->m_m110 + static_cast<FT>(tensor.get_m110()),
                this->m_m101 + static_cast<FT>(tensor.get_m101()),
                this->m_m011 + static_cast<FT>(tensor.get_m011()));
        }

        /**
         * @brief This function subtracts the provided tensor from the current tensor.
         *
         * This function creates a new tensor object and subtracts the provided tensor from the current tensor.
         *
         * @tparam T1 The type of the input tensor.
         * @param tensor The input tensor to subtract from the current tensor.
         * @return Tensor<FT> The new tensor object.
         */
        template <typename T1>
        inline Tensor<FT> operator-(const Tensor<T1> &tensor) const
        {
            return Tensor<FT>(
                this->m_m200 - static_cast<FT>(tensor.get_m200()),
                this->m_m020 - static_cast<FT>(tensor.get_m020()),
                this->m_m002 - static_cast<FT>(tensor.get_m002()),
                this->m_m110 - static_cast<FT>(tensor.get_m110()),
                this->m_m101 - static_cast<FT>(tensor.get_m101()),
                this->m_m011 - static_cast<FT>(tensor.get_m011()));
        }

        /**
         * @brief This function divides the current tensor by the provided scalar.
         *
         * @tparam T1  The type of the input scalar.
         * @param scalar The input scalar to divide the current tensor by.
         * @return Tensor<FT> The new tensor object.
         */
        template <typename T1>
        inline Tensor<FT> operator/(const T1 &scalar) const
        {
            return Tensor<FT>(
                this->m_m200 / static_cast<FT>(scalar),
                this->m_m020 / static_cast<FT>(scalar),
                this->m_m002 / static_cast<FT>(scalar),
                this->m_m110 / static_cast<FT>(scalar),
                this->m_m101 / static_cast<FT>(scalar),
                this->m_m011 / static_cast<FT>(scalar));
        }

        /**
         * @brief This function multiplies the current tensor by the provided scalar.
         *
         * @tparam T1 The type of the input scalar.
         * @param scalar The input scalar to multiply the current tensor by.
         * @return Tensor<FT> The new tensor object.
         */
        template <typename T1>
        inline Tensor<FT> operator*(const T1 &scalar) const
        {
            return Tensor<FT>(
                this->m_m200 * static_cast<FT>(scalar),
                this->m_m020 * static_cast<FT>(scalar),
                this->m_m002 * static_cast<FT>(scalar),
                this->m_m110 * static_cast<FT>(scalar),
                this->m_m101 * static_cast<FT>(scalar),
                this->m_m011 * static_cast<FT>(scalar));
        }

        /**
         * @brief This function compares the current tensor with the provided tensor for equality.
         *
         * @tparam T1 The type of the input tensor.
         * @param tensor The input tensor to compare with the current tensor.
         * @return true If the two tensors are equal.
         * @return false If the two tensors are not equal.
         */
        template <typename T1>
        inline bool operator==(const Tensor<T1> &tensor) const
        {
            return ((
                        m_m200 == tensor.get_m200()) &&
                    (m_m020 == tensor.get_m020()) &&
                    (m_m002 == tensor.get_m002()) &&
                    (m_m110 == tensor.get_m110()) &&
                    (m_m101 == tensor.get_m101()) &&
                    (m_m011 == tensor.get_m011()));
        }

        /**
         * @brief This function compares the current tensor with the provided tensor for inequality.
         *
         * @tparam T1 The type of the input tensor.
         * @param tensor The input tensor to compare with the current tensor.
         * @return true If the two tensors are not equal.
         * @return false If the two tensors are equal.
         */
        template <typename T1>
        inline bool operator!=(const Tensor<T1> &tensor) const
        {
            return ((
                        m_m200 != tensor.get_m200()) ||
                    (m_m020 != tensor.get_m020()) ||
                    (m_m002 != tensor.get_m002()) ||
                    (m_m110 != tensor.get_m110()) ||
                    (m_m101 != tensor.get_m101()) ||
                    (m_m011 != tensor.get_m011()));
        }

        /**
         * @brief This function checks if the current tensor is zero.
         *
         * @return true If the tensor is zero.
         * @return false If the tensor is not zero.
         */
        inline bool is_zero() const
        {
            return ((m_m200 == FT(0)) &&
                    (m_m020 == FT(0)) &&
                    (m_m002 == FT(0)) &&
                    (m_m110 == FT(0)) &&
                    (m_m101 == FT(0)) &&
                    (m_m011 == FT(0)));
        }

        /**
         * @brief This function calculates the eigen system of the tensor.
         *
         * @return true If the eigen system is calculated successfully.
         * @return false If the eigen system calculation fails.
         */
        bool calc_eigen_system()
        {
            FT a[3][3], d[3];

            a[0][0] = m_m200;
            a[0][1] = m_m110;
            a[0][2] = m_m101;

            a[1][0] = m_m110;
            a[1][1] = m_m020;
            a[1][2] = m_m011;

            a[2][0] = m_m101;
            a[2][1] = m_m011;
            a[2][2] = m_m002;

            if (eig_sys_3d(a, d) == false)
            {
                std::cerr << "[Tensor] Failed to calculate its eigen system!" << std::endl;
                return false;
            }
            else
            {
                m_e1.set(a[0][0], a[1][0], a[2][0]);
                m_e2.set(a[0][1], a[1][1], a[2][1]);
                m_e3.set(a[0][2], a[1][2], a[2][2]);

                m_lambda1 = d[0] > 0 ? d[0] : 0;
                m_lambda2 = d[1] > 0 ? d[1] : 0;
                m_lambda3 = d[2] > 0 ? d[2] : 0;

                return true;
            }
        }

    private:
        /**
         * @brief This function calculates the eigen system of the tensor.
         *
         * @param aa The tensor matrix.
         * @param di The eigenvalues.
         * @return true If the eigen system is calculated successfully.
         * @return false If the eigen system calculation fails.
         */
        bool eig_sys_3d(FT aa[][3], FT di[])
        {
            /* Commented by DT
       Algorithm Copied from Original Tensor Voting 3D version from IRIS
       !!!Not using Rene's copy of the eig_sys2d
       */

            /* inserted by Rene Dencker (edr@mip.sdu.dk) */
            // this was a global define in the old tensor system.
            // #define TOO_SMALL 1e-3
            const double TOO_SMALL = 1e-3;
            /* Adopted From Numerical Recipes in C (2nd Edition) pp. 474-475 :
            Householder reduction of a real, symmetric matrix a[0..2][0..2]. On output,
            a is replaced by the orthogonal matrix Q effecting the transformation.
            d[0..2] returns the diagonal elements of the tridiagonal matrix,
            and e[0..2] the off-diagonal elements, with e[0] = 0. Several
            statements, as noted in comments, can be omitted if only eigenvalues are
            to be found, in which case a contains no useful information on output.
            Otherwise they are to be included. */
            double scale, hh, h, g, f, a[3][3], d[3], e[3];

            a[0][0] = aa[0][0];
            a[0][1] = aa[0][1];
            a[0][2] = aa[0][2];
            a[1][0] = aa[1][0];
            a[1][1] = aa[1][1];
            a[1][2] = aa[1][2];
            a[2][0] = aa[2][0];
            a[2][1] = aa[2][1];
            a[2][2] = aa[2][2];
            d[0] = di[0];
            d[1] = di[1];
            d[2] = di[2];
            // i = 2; l = 1;
            h = 0.0;
            scale = fabs(a[2][0]) + fabs(a[2][1]);
            if (scale == 0.0)
                e[2] = a[2][1];
            else
            {
                a[2][0] /= scale;
                a[2][1] /= scale;
                h = a[2][0] * a[2][0] + a[2][1] * a[2][1];
                f = a[2][1];
                g = f >= 0 ? -sqrt(h) : sqrt(h);
                e[2] = scale * g;
                h -= f * g;
                a[2][1] = f - g;

                a[0][2] = a[2][0] / h;
                e[0] = (a[0][0] * a[2][0] + a[1][0] * a[2][1]) / h;
                a[1][2] = a[2][1] / h;
                e[1] = (a[1][0] * a[2][0] + a[1][1] * a[2][1]) / h;
                f = e[0] * a[2][0] + e[1] * a[2][1];

                hh = f / (h + h);
                f = a[2][0];
                e[0] = g = e[0] - hh * f;
                a[0][0] -= f * e[0] + g * a[2][0];
                f = a[2][1];
                e[1] = g = e[1] - hh * f;
                a[1][0] -= f * e[0] + g * a[2][0];
                a[1][1] -= f * e[1] + g * a[2][1];
            }
            d[2] = h;

            // i = 1; l = 0;
            e[1] = a[1][0];
            d[1] = 0.0;

            // Next statement can be omitted if eigenvectors not wanted
            // i=0; l=-1;
            d[0] = a[0][0];
            a[0][0] = 1.0;

            // i=1; l=0;
            if (d[1])
                a[0][0] -= (a[1][0] * a[0][0]) * a[0][1];
            d[1] = a[1][1];
            a[1][1] = 1.0;
            a[0][1] = a[1][0] = 0.0;

            // i=2; l=1;
            if (d[2])
            {
                g = a[2][0] * a[0][0] + a[2][1] * a[1][0];
                a[0][0] -= g * a[0][2];
                a[1][0] -= g * a[1][2];
                g = a[2][0] * a[0][1] + a[2][1] * a[1][1];
                a[0][1] -= g * a[0][2];
                a[1][1] -= g * a[1][2];
            }
            d[2] = a[2][2];
            a[2][2] = 1.0;
            a[0][2] = a[2][0] = 0.0;
            a[1][2] = a[2][1] = 0.0;

            /* Adopted From Numerical Recipes in C (2nd Edition) pp. 480-481 :
            QL algorithm with implicit shifts, to determine the eigenvalues and
            eigenvectors of a real, symmetric, tridiagonal matrix, or of a real,
            symmetric matrix previously reduced by tred2.  On input, d[1..n] contains
            the diagonal elements of the tridiagonal matrix. On output, it returns the
            eigenvalues.  The vector e[1..n] inputs the subdiagonal elements of the
            tridiagonal matrix, with e[1] arbitrary.  On output e is destroyed.  When
            finding only the eigenvalues, several lines may be omitted, as noted in the
            comments.  If the eigenvectors of a tridiagonal matrix are desired, the
            matrix a[1..n][1..n] is input as the identity matrix.  If the eigenvectors
            of a matrix that has been reduced by tred2 are required, then z is input as
            the matrix output by tred2. In either case, the kth column of z returns the
            normalized eigenvector corresponding to d[k]. */

            int m, l, iter, i, k;
            double s, r, p, dd, c, b;

            e[0] = e[1];
            e[1] = e[2];
            e[2] = 0.0;

            for (l = 0; l <= 2; l++)
            {
                iter = 0;
                do
                {
                    for (m = l; m <= 1; m++)
                    {
                        dd = fabs(d[m]) + fabs(d[m + 1]);
                        if (fabs(e[m]) + dd == dd)
                            break;
                    }
                    if (m != l)
                    {
                        if (iter++ == 30)
                        {
                            // std::cout << "Too many iterations in TQLI" << std::endl;
                            return false;
                        }
                        g = (d[l + 1] - d[l]) / (2.0 * e[l]);
                        r = sqrt((g * g) + 1.0);
                        g = d[m] - d[l] + e[l] / (g + sign(r, g));
                        s = c = 1.0;
                        p = 0.0;
                        for (i = m - 1; i >= l; i--)
                        {
                            f = s * e[i];
                            b = c * e[i];
                            if (fabs(f) >= fabs(g))
                            {
                                c = g / f;
                                r = sqrt((c * c) + 1.0);
                                e[i + 1] = f * r;
                                c *= (s = 1.0 / r);
                            }
                            else
                            {
                                s = f / g;
                                r = sqrt((s * s) + 1.0);
                                e[i + 1] = g * r;
                                s *= (c = 1.0 / r);
                            }
                            g = d[i + 1] - p;
                            r = (d[i] - g) * s + 2.0 * c * b;
                            p = s * r;
                            d[i + 1] = g + p;
                            g = c * r - b;
                            // Next loop can be omitted if eigenvectors not wanted
                            for (k = 0; k <= 2; k++)
                            {
                                f = a[k][i + 1];
                                a[k][i + 1] = s * a[k][i] + c * f;
                                a[k][i] = c * a[k][i] - s * f;
                            }
                        }
                        d[l] = d[l] - p;
                        e[l] = g;
                        e[m] = 0.0;
                    }
                } while (m != l);
            }

            // sort the eigenvalues and eigenvectors
            double max_val;
            int max_index;
            for (i = 0; i <= 1; i++)
            {
                max_val = d[i];
                max_index = i;
                for (k = i + 1; k <= 2; k++)
                {
                    if (max_val < d[k])
                    {
                        max_val = d[k];
                        max_index = k;
                    }
                }
                if (max_index != i)
                {
                    e[0] = d[i];
                    d[i] = d[max_index];
                    d[max_index] = e[0];
                    e[0] = a[0][i];
                    a[0][i] = a[0][max_index];
                    a[0][max_index] = e[0];
                    e[0] = a[1][i];
                    a[1][i] = a[1][max_index];
                    a[1][max_index] = e[0];
                    e[0] = a[2][i];
                    a[2][i] = a[2][max_index];
                    a[2][max_index] = e[0];
                }
            }
            aa[0][0] = (FT)a[0][0];
            aa[0][1] = (FT)a[0][1];
            aa[0][2] = (FT)a[0][2];
            aa[1][0] = (FT)a[1][0];
            aa[1][1] = (FT)a[1][1];
            aa[1][2] = (FT)a[1][2];
            aa[2][0] = (FT)a[2][0];
            aa[2][1] = (FT)a[2][1];
            aa[2][2] = (FT)a[2][2];
            di[0] = (FT)d[0];
            di[1] = (FT)d[1];
            di[2] = (FT)d[2];

            return true;
        }

        inline FT sign(FT a, FT b)
        {
            return (b < 0 ? -fabs(a) : fabs(a));
        }
    }; //!- template class Tensor

} //!- namespace mm.

#endif //!- MM_BASIC_TENSOR_H