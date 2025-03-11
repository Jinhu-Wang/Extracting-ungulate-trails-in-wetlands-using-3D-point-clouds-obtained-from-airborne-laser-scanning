#ifndef MM_BASIC_TENSOR_VOTE_H
#define MM_BASIC_TENSOR_VOTE_H

// System
#include <vector>

// Local
#include "tensor.hpp"
#include <iostream>

namespace mm
{

    /**
     * @brief This class represents a tensor vote in a tensor voting system.
     *
     * The class is able to hold the eigenvalues and eigenvectors of the tensor matrix.
     * Also, this class defines the components of the tensor vote, such as stick, plate, and sphere tensors.
     * The stickness, surfaceness, and sphericity of the tensor are also defined in this class.
     *
     * This class provides a representation of a tensor vote with member attributes for tensor matrix values,
     *
     * @tparam FT The type of the tensor matrix, must be floating point type.
     */
    template <typename FT>
    class TensorVote : public Tensor<FT>
    {
        /*  -------------		Member Attributes					  -------------- */
    public:
    protected:
        /// @brief The stick tensor component of the tensor vote.
        Tensor<FT> m_stick_tensor;
        /// @brief The plate tensor component of the tensor vote.
        Tensor<FT> m_plate_tensor;
        /// @brief The sphere tensor component of the tensor vote.
        Tensor<FT> m_sphere_tensor;

        /// @brief The stickness of the tensor (L1 -L2).
        FT m_stickness;
        // The surfaceness of the tensor - [L2-L3];
        FT m_surfaceness;
        // The pointness of the tensor - [L3];
        FT m_sphericity;
        // The flag to indicate if the tensor is splitted;
        bool m_splitted;

        /*  -------------		Constructors and Destructor		  -------------- */
    public:
        /**
         * @brief Construct a new Tensor Vote object
         *
         *  This constructor initializes a TensorVote object and initializes the stick,
         * plate, and sphere tensors to their default zero states. It also initializes the
         * stickness, surfaceness, and sphericity to zero.
         *
         * @note The tensor is initialized to 0.
         */
        TensorVote()
            : Tensor<FT>(),
              m_stick_tensor(Tensor<FT>()),
              m_plate_tensor(Tensor<FT>()),
              m_sphere_tensor(Tensor<FT>()),
              m_stickness(0),
              m_surfaceness(0),
              m_sphericity(0),
              m_splitted(false)
        {
        }

        /**
         * @brief Construct a new Tensor Vote object
         *
         * This constructor initializes a TensorVote object using the provided tensor
         * components and initializes additional member variables for stickness, surfaceness,
         * and sphericity.
         *
         * @param M200 The upper left value in the tensor matrix (X^2)
         * @param M020 The center value in the tensor matrix (Y^2)
         * @param M002 The lower right value in the tensor matrix (Z^2)
         * @param M110 The tensor matrix (XY)
         * @param M101 The tensor matrix (XZ)
         * @param M011 The tensor matrix (YZ)
         */
        template <typename T1>
        TensorVote(T1 M200, T1 M020, T1 M002, T1 M110, T1 M101, T1 M011)
            : Tensor<FT>(static_cast<FT>(M200),
                         static_cast<FT>(M020),
                         static_cast<FT>(M002),
                         static_cast<FT>(M110),
                         static_cast<FT>(M101),
                         static_cast<FT>(M011)),
              m_stick_tensor(Tensor<FT>()),
              m_plate_tensor(Tensor<FT>()),
              m_sphere_tensor(Tensor<FT>()),
              m_stickness(0),
              m_surfaceness(0),
              m_sphericity(0),
              m_splitted(false)
        {
        }

        /**
         * @brief Construct a new TensorVote object from a given tensor.
         *
         * This constructor initializes a TensorVote object by copying the values
         * from the provided tensor. It also initializes the stick, plate, and sphere
         * tensors to their default states and sets the stickness, surfaceness, and
         * sphericity to zero.
         *
         * @tparam T1  The type of the input tensor.
         * @param tensor The input tensor from which to initialize the TensorVote object.
         */
        template <typename T1>
        inline TensorVote(const Tensor<T1> &tensor)
            : Tensor<FT>(static_cast<FT>(tensor.get_m200()),
                         static_cast<FT>(tensor.get_m020()),
                         static_cast<FT>(tensor.get_m002()),
                         static_cast<FT>(tensor.get_m110()),
                         static_cast<FT>(tensor.get_m101()),
                         static_cast<FT>(tensor.get_m011())),
              m_stick_tensor(Tensor<FT>()),
              m_plate_tensor(Tensor<FT>()),
              m_sphere_tensor(Tensor<FT>()),
              m_stickness(0),
              m_surfaceness(0),
              m_sphericity(0),
              m_splitted(false)
        {
        }

        /**
         * @brief Construct a new Tensor Vote object
         *
         * This constructor initializes a TensorVote object using the provided direction vector.
         * It initializes the stick, plate, and sphere tensors to their default states and sets the
         * stickness, surfaceness, and sphericity to zero.
         *
         * @tparam T1 The type of the input direction vector.
         * @param direction The input dominant tensor direction vector.
         */
        template <typename T1>
        inline TensorVote(const Vec<3, T1> &direction)
            : Tensor<FT>(static_cast<FT>(direction)),
              m_stick_tensor(),
              m_plate_tensor(),
              m_sphere_tensor(),
              m_stickness(0),
              m_surfaceness(0),
              m_sphericity(0),
              m_splitted(false)
        {
        }

        /**
         * @brief This is the copy constructor for the TensorVote object.
         *
         * @tparam T1 The data type of the input tensor.
         * @param vote The input tensor vote object from which to initialize the current tensor vote object.
         *
         * @return TensorVote<FT>& The current tensor vote object.
         */
        template <typename T1>
        inline TensorVote<FT> &operator=(const TensorVote<T1> &vote)
        {
            if (this == reinterpret_cast<const TensorVote<FT> *>(&vote))
                return *this; // Avoid self-assignment

            this->set_m200(static_cast<FT>(vote.get_m200()));
            this->set_m020(static_cast<FT>(vote.get_m020()));
            this->set_m002(static_cast<FT>(vote.get_m002()));
            this->set_m110(static_cast<FT>(vote.get_m110()));
            this->set_m101(static_cast<FT>(vote.get_m101()));
            this->set_m011(static_cast<FT>(vote.get_m011()));

            return *this;
        }

        /**
         * @brief Set the tensor object with the provided tensor.
         *
         * @param tensor The input tensor object to set the current tensor object with.
         *
         * @return templat<typename T1> The current tensor object.
         */
        template <typename T1>
        inline void set(const Tensor<T1> &tensor)
        {
            this->set_m200(static_cast<FT>(tensor.get_m200()));
            this->set_m020(static_cast<FT>(tensor.get_m020()));
            this->set_m002(static_cast<FT>(tensor.get_m002()));
            this->set_m110(static_cast<FT>(tensor.get_m110()));
            this->set_m101(static_cast<FT>(tensor.get_m101()));
            this->set_m011(static_cast<FT>(tensor.get_m011()));
        }

        /**
         * @brief Adds the provided tensor to the current tensor.
         *
         * @tparam T1 The type of the input tensor.
         * @param vote The input tensor to add to the current tensor.
         * @return TensorVote<FT> The returned tensor object.
         */
        template <typename T1>
        inline TensorVote<FT> operator+(const TensorVote<T1> &vote) const
        {
            return TensorVote<FT>(
                this->get_m200() + static_cast<FT>(vote.get_m200()),
                this->get_m020() + static_cast<FT>(vote.get_m020()),
                this->get_m002() + static_cast<FT>(vote.get_m002()),
                this->get_m110() + static_cast<FT>(vote.get_m110()),
                this->get_m101() + static_cast<FT>(vote.get_m101()),
                this->get_m011() + static_cast<FT>(vote.get_m011()));
        }

        /**
         * @brief  This function adds the provided tensor to the current tensor.
         *
         * @tparam T1 The  type of the input tensor.
         * @param vote The input tensor to add to the current tensor.
         * @return TensorVote<FT>& The returned tensor object.
         */
        template <typename T1>
        inline TensorVote<FT> &operator+=(const TensorVote<T1> &vote)
        {
            this->set_m200(this->get_m200() + static_cast<FT>(vote.get_m200()));
            this->set_m020(this->get_m020() + static_cast<FT>(vote.get_m020()));
            this->set_m002(this->get_m002() + static_cast<FT>(vote.get_m002()));
            this->set_m110(this->get_m110() + static_cast<FT>(vote.get_m110()));
            this->set_m101(this->get_m101() + static_cast<FT>(vote.get_m101()));
            this->set_m011(this->get_m011() + static_cast<FT>(vote.get_m011()));
            return *this;
        }

        bool calc_eigen_system()
        {
            if (Tensor<FT>::calc_eigen_system())
            {
                return true;
            }
            else
            {
                LOG(ERROR) << "The tensor vote eigen system calculation failed.";
                return false;
            }
        }

    public:
        /**
         * @brief This function splits the tensor vote into stick, plate, and sphere tensors.
         *
         * It also calcuate the stickness, surfaceness, and sphericity of the tensor vote.
         *
         * @param CalcEigenSyst The flag to indicate if the eigen system should be calculated.
         * @return true If the tensor vote was successfully split.
         * @return false If the tensor vote could not be split.
         */
        bool split(bool CalcEigenSyst = true)
        {
            if (CalcEigenSyst == true)
            {
                if (calc_eigen_system() == false)
                {
                    // LOG(ERROR) << "The tensor vote failed to calculate its eigen system.";
                    return false;
                }
            }

            // Uniformed version.
            double sum_lambdas = this->m_lambda1 + this->m_lambda2 + this->m_lambda3;
            this->m_stickness = (this->m_lambda1 - this->m_lambda2) / sum_lambdas;
            this->m_surfaceness = (this->m_lambda2 - this->m_lambda3) / sum_lambdas;
            this->m_sphericity = this->m_lambda3 / sum_lambdas;

            // Compute the three components of the compound tensor;

            // Stick component;
            this->m_stick_tensor = Tensor<FT>(this->m_e1);
            this->m_stick_tensor *= this->m_stickness;

            // Plate component;
            this->m_plate_tensor = Tensor<FT>(this->m_e1);
            this->m_plate_tensor += Tensor<FT>(this->m_e2);
            this->m_plate_tensor *= this->m_surfaceness;

            // Sphere component;
            this->m_sphere_tensor = Tensor<FT>(this->m_e1);
            this->m_sphere_tensor += Tensor<FT>(this->m_e2);
            this->m_sphere_tensor += Tensor<FT>(this->m_e3);
            this->m_sphere_tensor *= this->m_sphericity;

            // calc_components_eigen_systems();

            this->m_splitted = true;

            // LOG(INFO) << "The tensor vote was successfully split into its components.";

            return true;
        }

        /**
         * @brief This calculates the eigen system for the components of the tensor vote.
         *
         * This function calculates the eigen system for the stick, plate, and sphere components of
         * the tensor vote.
         *
         * @return true If the eigen system was calculated successfully.
         * @return false If the eigen system could not be calculated.
         */
        bool calc_components_eigen_systems()
        {
            if (this->m_stick_tensor.calc_eigen_system() == false)
            {
                // LOG(ERROR) << "The stick component failed to calculate its eigen system!";
                return false;
            }
            if (this->m_plate_tensor.calc_eigen_system() == false)
            {
                // LOG(ERROR) << "The plate component failed to calculate its eigen system!";
                return false;
            }
            if (this->m_sphere_tensor.calc_eigen_system() == false)
            {
                // LOG(ERROR) << "The sphere component failed to calculate its eigen system!";
                return false;
            }
            return true;
        }

        /**
         * @brief Get the stick component object
         *
         * @return const Tensor<FT>& The stick component of the tensor.
         */
        inline const Tensor<FT> &get_stick_component() const
        {
            return this->m_stick_tensor;
        }

        /**
         * @brief Get the plate component object
         *
         * @return const Tensor<FT>& The plate component of the tensor.
         */
        inline const Tensor<FT> &get_plate_component() const
        {
            return this->m_plate_tensor;
        }

        /**
         * @brief Get the sphere component object
         *
         * @return const Tensor<FT>&  The sphere component of the tensor.
         */
        inline const Tensor<FT> &get_sphere_component() const
        {
            return this->m_sphere_tensor;
        }

        /**
         * @brief Get the stickness object
         *
         * @return const FT& The stickness of the tensor.
         */
        inline const FT &get_stickness() const
        {
            return this->m_stickness;
        }

        /**
         * @brief Get the plateness object
         *
         * @return const FT& The plateness of the tensor.
         */
        inline const FT &get_surfaceness() const
        {
            return this->m_surfaceness;
        }

        /**
         * @brief Get the sphericity object
         *
         * @return const FT& The sphericity of the tensor.
         */
        inline const FT &get_sphericity() const
        {
            return this->m_sphericity;
        }

        /**
         *  @brief Get the base object object
         *
         * @return Tensor<FT>& The base object of the tensor.
         */
        inline Tensor<FT> &get_base_object()
        {
            return *this;
        }

        /**
         * @brief Get the base object object
         *
         * @return const Tensor<FT>& The base object of the tensor.
         */
        inline const Tensor<FT> &get_base_object() const
        {
            return *this;
        }

    }; //!- template class TensorVote.

} //!- namespace mm.

#endif //!- MM_BASIC_TENSOR_VOTE_H