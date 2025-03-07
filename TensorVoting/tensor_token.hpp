#ifndef MM_BASIC_TENSOR_TOKEN_H
#define MM_BASIC_TENSOR_TOKEN_H

// System
#include <cmath>
// #include<tuple>
#include <vector>

// Local
#include <miemie/basic/quat.h>
#include <miemie/basic/vec.h>

#include <miemie/util/logging.h>

#include "tensor_vote.hpp"

namespace mm
{

    template <typename FT>
    class TensorToken : public Vec<3, FT>
    {
    public:
        // IF the position is initialized;
        bool m_position_available;
        // If the direction is available at this position;
        bool m_dir_available;
        // If the tensor is available at this position;
        bool m_voter_available;
        // If the tensor is available at this position;
        bool m_votee_available;
        // The dominant direction (stick or plane? ) at this position;
        Vec<3, FT> m_dir;
        // The tensor associated with this position - to vote;
        TensorVote<FT> m_voter;
        // The tensor associated with this position - to receive vote;
        TensorVote<FT> m_votee;
        // The saliency at this position;
        FT m_saliency;
        // An internal variable (vector sum) for end point saliency calculation.
        Vec<3, FT> m_end_point_saliency;
        // If the tensor token is an outlier;
        bool m_is_outlier;
        // The number of neighbours;
        size_t m_num_neighbours;

    public:
        /**
         * @brief Construct a new Tensor Vote object.
         *
         * This constructor initializes a TensorVote object and initializes the stick,
         * plate, and sphere tensors to their default states. It also initializes the
         * stickness, surfaceness, and sphericity to zero.
         *
         * @note   The tensor is initialized to 0.
         */
        TensorToken()
            : Vec<3, FT>(),
              m_dir_available(false),
              m_position_available(false),
              m_voter_available(false),
              m_votee_available(false),
              m_dir(Vec<3, FT>()),
              m_voter(TensorVote<FT>()),
              m_votee(TensorVote<FT>()),
              m_saliency(FT(0)),
              m_end_point_saliency(Vec<3, FT>()),
              m_is_outlier(false),
              m_num_neighbours(0)
        {
        }

        /**
         * @brief Construct a new Tensor Token object
         *
         * @param p The position of the tensor token.
         */
        explicit TensorToken(const Vec<3, FT> &p)
            : Vec<3, FT>(p),
              m_dir_available(false),
              m_voter_available(false),
              m_votee_available(false),
              m_dir(Vec<3, FT>()),
              m_voter(TensorVote<FT>()),
              m_votee(TensorVote<FT>()),
              m_saliency(0),
              m_end_point_saliency(Vec<3, FT>()),
              m_is_outlier(false),
              m_num_neighbours(0)
        {
        }

        /**
         * @brief Construct a new Tensor Token object with a given position and orientation.
         *
         * @param x The x-coordinate of the position.
         * @param y The y-coordinate of the position.
         * @param z The z-coordinate of the position.
         * @param dir The direction of the tensor.
         */
        TensorToken(FT x, FT y, FT z, const Vec<3, FT> &dir)
            : Vec<3, FT>(x, y, z),
              m_position_available(true),
              m_dir_available(true),
              m_voter_available(true),
              m_votee_available(false),
              m_dir(dir),
              m_voter(dir),
              m_votee(TensorToken<FT>()),
              m_saliency(0),
              m_end_point_saliency(Vec<3, FT>()),
              m_is_outlier(false),
              m_num_neighbours(0)
        {
            // Since the voter is initialized with the direction, we need to split it.
            this->m_voter.split();
        }

        /**
         * @brief Construct a new Tensor Token object
         *
         * @param p  This is the position of the tensor token.
         * @param dir This is the dominant direction of the tensor token.
         */
        TensorToken(const Vec<3, FT> &p, const Vec<3, FT> &dir)
            : Vec<3, FT>(p),
              m_position_available(true),
              m_dir_available(true),
              m_voter_available(true),
              m_votee_available(false),
              m_dir(dir),
              m_voter(dir),
              m_votee(TensorVote<FT>()),
              m_saliency(0),
              m_end_point_saliency(Vec<3, FT>()),
              m_is_outlier(false),
              m_num_neighbours(0)
        {
            // Since the voter is initialized with the direction, we need to split it.
            this->m_voter.split();
        }

        /**
         * @brief Construct a new Tensor Token object with a given position, orientation and tensor.
         *
         * @param x The x-coordinate of the position.
         * @param y The y-coordinate of the position.
         * @param z The z-coordinate of the position.
         * @param dir  The direction of the tensor.
         * @param tensor The tensor associated with the position.
         */
        TensorToken(FT x, FT y, FT z, const Vec<3, FT> &dir, const Tensor<FT> &tensor)
            : Vec<3, FT>(x, y, z),
              m_position_available(true),
              m_dir_available(true),
              m_voter_available(true),
              m_votee_available(false),
              m_dir(dir),
              m_voter(tensor),
              m_votee(TensorVote<FT>()),
              m_saliency(0),
              m_end_point_saliency(Vec<3, FT>()),
              m_is_outlier(false),
              m_num_neighbours(0)
        {
            // Since the voter is initialized with the direction, we need to split it.
            m_voter.split();
        }

        /**
         * @brief Construct a new Tensor Token object with a given position and tensor.
         *
         * @param p The position of the tensor token.
         * @param tensor The tensor associated with the position.
         */
        TensorToken(const Vec<3, FT> &p, const Tensor<FT> &tensor)
            : Vec<3, FT>(p),
              m_position_available(true),
              m_dir_available(false),
              m_voter_available(true),
              m_votee_available(false),
              m_dir(Vec<3, FT>()),
              m_voter(tensor),
              m_votee(),
              m_saliency(0),
              m_end_point_saliency(Vec<3, FT>()),
              m_is_outlier(false),
              m_num_neighbours(0)
        {
            // Since the voter is initialized with the direction, we need to split it.
            m_voter.split();
            // Then, we initialize the direction with the dominant direction.
            this->m_dir.set(m_voter.get_e1());
        }

        /**
         * @brief Construct a new Tensor Token object
         *
         * This function initializes a TensorToken object by copying the
         * values from the provided TensorToken object.
         *
         * @param token
         */
        TensorToken(const TensorToken<FT> &token)
            : Vec<3, FT>(token),
              m_position_available(token.m_position_available),
              m_dir_available(token.m_dir_available),
              m_voter_available(token.m_voter_available),
              m_votee_available(token.m_votee_available),
              m_dir(token.m_dir),
              m_voter(token.m_voter),
              m_votee(token.m_votee),
              m_saliency(token.m_saliency),
              m_end_point_saliency(token.m_end_point_saliency),
              m_is_outlier(token.m_is_outlier),
              m_num_neighbours(token.m_num_neighbours)
        {
        }

        /**
         * @brief Destroy the Tensor Token object
         *
         */
        ~TensorToken() = default;

    public:
        /**
         * @brief Overload the assignment operator.
         *
         * @param token The tensor token to copy.
         * @return TensorToken<FT>& The current tensor token object.
         */
        inline TensorToken<FT> &operator=(const TensorToken<FT> &token)
        {
            if (this != &token)
            {
                Vec<3, FT>::operator=(token);
                m_dir_available = token.m_dir_available;
                m_position_available = token.m_position_available;
                m_voter_available = token.m_voter_available;
                m_votee_available = token.m_votee_available;
                m_dir = token.m_dir;
                m_voter = token.m_voter;
                m_votee = token.m_votee;
                m_saliency = token.m_saliency;
                m_end_point_saliency = token.m_end_point_saliency;
                m_is_outlier = token.m_is_outlier;
                m_num_neighbours = token.m_num_neighbours;
            }
            return *this;
        }

        /*  -------------			Getters and Setters					  -------------- */
    public:
        /**
         * @brief Set the position of the tensor token.
         * @note The direciton is unavailable.
         * @param p The position of the tensor token.
         */
        template <typename T1>
        inline void set(const Vec<3, T1> &p)
        {
            Vec<3, FT>::set(p.get_x(), p.get_y(), p.get_z());

            m_dir.set(FT(0), FT(0), FT(0));
            m_dir_available = false;
            m_voter.set(TensorVote<FT>(0, 0, 0, 0, 0, 0));
            m_votee.set(TensorVote<FT>(0, 0, 0, 0, 0, 0));
            m_voter_available = false;
            m_votee_available = false;
            m_saliency = FT(0);
            m_is_outlier = false;
            m_num_neighbours = 0;
            m_end_point_saliency.set(FT(0), FT(0), FT(0));
        }

        /**
         * @brief Set the position of the tensor token.
         *
         * @param x The x-coordinate of the position.
         * @param y The y-coordinate of the position.
         * @param z The z-coordinate of the position.
         */
        inline void set(const FT &x, const FT &y, const FT &z)
        {
            Vec<3, FT>::set(x, y, z);

            m_dir.set(FT(0), FT(0), FT(0));
            m_dir_available = false;
            m_voter.set(Tensor<FT>(0, 0, 0, 0, 0, 0));
            m_votee.set(Tensor<FT>(0, 0, 0, 0, 0, 0));
            m_voter_available = false;
            m_votee_available = false;
            m_saliency = FT(0);
            m_is_outlier = false;
            m_num_neighbours = 0;
            m_end_point_saliency.set(FT(0), FT(0), FT(0));
        }

        /**
         * @brief  Set the position and direction of the tensor token.
         *
         * @param p The position of the tensor token.
         * @param dir The direction of the tensor token.
         */
        inline void set(const Vec<3, FT> &p, const Vec<3, FT> &dir)
        {
            Vec<3, FT>::set(p.get_x(), p.get_y(), p.get_z());
            m_dir = dir;
            m_dir_available = true;
            m_voter.set(dir);
            // Since just initialized, we need to split the tensor.
            m_voter.split();
            m_voter_available = true;

            m_votee.set(Tensor<FT>(0, 0, 0, 0, 0, 0));
            m_votee_available = false;

            m_saliency = FT(0);
            m_is_outlier = false;
            m_num_neighbours = 0;
            m_end_point_saliency.set(FT(0), FT(0), FT(0));
        }

        /**
         * @brief This sets the position and tensor of the tensor token.
         *
         * @param p The position of the tensor token.
         * @param tensor The tensor associated with the position.
         */
        inline void set(const Vec<3, FT> &p, const Tensor<FT> &tensor)
        {
            Vec<3, FT>::set(p.get_x(), p.get_y(), p.get_z());
            m_voter = tensor;
            m_voter_available = true;
            m_dir = Vec<3, FT>(0, 0, 0);
            m_dir_available = false;

            if (m_voter.split())
            {
                m_dir.set(m_voter.get_e1());
                m_dir_available = true;
            }
            m_votee.set(Tensor<FT>(0, 0, 0, 0, 0, 0));
            m_votee_available = false;
            m_saliency = FT(0);
            m_is_outlier = false;
            m_num_neighbours = 0;
            m_end_point_saliency.set(FT(0), FT(0), FT(0));
        }

        /**
         * @brief Get the position of the tensor token.
         *
         * @return const Vec<3,FT> The position of thetensor token.
         */
        inline const Vec<3, FT> get_position() const
        {
            return Vec<3, FT>(this->get_x(), this->get_y(), this->get_z());
        }

        /**
         * @brief Get the dir object of the tensor token.
         *
         * @return const Vec<3, FT>&  The direction of the tensor token.
         */
        inline const Vec<3, FT> &get_dir() const
        {
            return this->m_dir;
        }

        /**
         * @brief Set the dir object of the tensor token.
         *
         * @param dir The direction of the tensor token.
         */
        inline void set_dir(const Vec<3, FT> &dir)
        {
            this->m_dir = dir;
            this->m_voter.set(dir);
        }

        /**
         * @brief Set the outlier object
         *
         * @param is_outlier The flag to set the outlier.
         */
        inline void set_outlier(bool is_outlier)
        {
            m_is_outlier = is_outlier;
        }

        /**
         * @brief Get the outlier object
         *
         * @return true If the tensor token is an outlier, return true.
         * @return false If the tensor token is not an outlier, return false.
         */
        inline bool is_outlier() const
        {
            return m_is_outlier;
        }

        /**
         * @brief Set the num neighbours object
         *
         * @param num_neighbours The number of neighbours.
         */
        inline void set_num_neighbours(size_t num_neighbours)
        {
            m_num_neighbours = num_neighbours;
        }
        /**
         * @brief Get the num neighbours object
         *
         * @return size_t The number of neighbours.
         */
        inline size_t get_num_neighbours() const
        {
            return m_num_neighbours;
        }

        /**
         * @brief Add a direction to the tensor token.
         *
         * @param dir The direction to add.
         */
        inline void add_dir(const Vec<3, FT> &dir)
        {
            this->m_dir += dir;
            this->m_dir_available = true;
        }

        /**
         * @brief Get the tensor object of the tensor token.
         *
         * @return TensorVote<FT>&  The tensor voter of the tensor token.
         */
        inline TensorVote<FT> get_voter() const
        {
            return this->m_voter;
        }

        /**
         * @brief Get the tensor votee object
         *
         * @return TensorVote<FT>& The tensor votee of the tensor token.
         */
        inline TensorVote<FT> &get_votee()
        {
            return this->m_votee;
        }

        /**
         * @brief Set the tensor object of the tensor token.
         *
         * @param tensor The tensor vote of the tensor token.
         * @param SplitTensor The flag to split the tensor.
         */
        inline void set_voter(const Tensor<FT> &tensor, bool SplitTensor = true)
        {
            this->m_voter = tensor;
            if (SplitTensor)
            {
                this->m_voter.split();
            }
            this->m_voter_available = true;
        }
        /**
         * @brief To check if the direction is available.
         *
         * @return true If the direction is available, return true.
         * @return false If the direction is not available, return false.
         */
        inline bool dir_available() const
        {
            return this->m_dir_available;
        }

        /**
         * @brief To check if the tensor is available.
         *
         * @return true If the tensor is available, return true.
         * @return false If the
         */
        inline bool tensor_available() const
        {
            return this->m_voter_available;
        }

        /**
         * @brief To delete the direction vector.
         *
         */
        inline void delete_dir()
        {
            this->m_dir.set(0, 0, 0);
            this->m_dir_available = false;
        }

        /**
         * @brief To add a tensor to the tensor vote.
         *
         * @param tensor The tensor to add.
         * @param SplitTensor The flag to split the tensor.
         */
        inline void add_tensor(const Tensor<FT> &tensor, bool SplitTensor = true)
        {
            this->m_voter += tensor;
            if (SplitTensor)
            {
                this->m_tensor.split();
            }
            this->m_voter_available = true;
        }

        /**
         * @brief Split the tensor value in [m_voter] to stick, plate and ball components.
         *      If the tensor is not available, return false.
         *
         * @param CalcComponentEigenSyst If true, calculate the eigen system of the components.
         * @return true if the tensor is available, otherwise return false.
         * @return false if the tensor is not available or if eigensystem could be calculated.
         */
        bool split_tensor(bool CalcComponentEigenSyst = true)
        {
            if (m_voter_available)
            {
                m_voter.split(CalcComponentEigenSyst);
                return true;
            }
            return false;
        }

        /**
         * @brief To verify if the direction of the tensor is coherent with the significant direction of the node.
         *
         * @param direction The direction to verify.
         * @return true If the direction is coherent with the significant direction of the node, return true.
         * @return false If the direction is not coherent with the significant direction of the node, return false.
         */
        bool verify_direction(Vec<3, FT> direction)
        {
            if (m_dir_available)
            {
                return (m_dir * direction) > 0;
            }
            return false;
        }

        /**
         * @brief This function calculates the stick voting direction of of a tensor token from a voter to votee.
         *
         * @note I directly use the direction of the stick tensor as the stick voting direction.
         * @param votee The tensor token to receive the vote.
         * @param stick_voting_direction The calcualted stick voting direction.
         * @param theta The calculated angle between the stick voting direction and the direction of v = votee - voter.
         * @param angle_threshold The threshold to check if the angle is larger than the threshold.
         * @return true If the stick voting direction and theta are calculated successfully.
         * @return false If the stick voting direction and theta are not calculated.
         */
        bool calc_stick_voting_direction(const TensorToken<FT> &votee,
                                         Vec<3, FT> &stick_voting_direction,
                                         FT &theta,
                                         const FT angle_threshold = M_PI / 4.0)
        {
            if (m_voter_available)
            {
                FT alpha{0.0};
                Vec<3, FT> ref_dir{Vec<3, FT>()};

                // Get the vector from voter to the votee.
                Vec<3, FT> vec_voter_to_votee = votee - this->get_position();

                // Define two directions, i.e. forward and backward.
                Vec<3, FT> voting_dir_forward = this->m_voter.get_e1();
                Vec<3, FT> voting_dir_backward = -this->m_voter.get_e1();

                FT theta_forward = angle(vec_voter_to_votee, voting_dir_forward);
                FT theta_backward = angle(vec_voter_to_votee, voting_dir_backward);

                if (theta_forward < theta_backward)
                {
                    alpha = theta_forward;
                    ref_dir = this->m_voter.get_e1();
                }
                else
                {
                    alpha = theta_backward;
                    ref_dir = -this->m_voter.get_e1();
                }

                // We calculate the direction to the votee that are only within the angle threshold.
                if (alpha <= angle_threshold)
                {
                    Vec<3, FT> rot_axis = cross(vec_voter_to_votee, ref_dir);
                    Quat<FT> q(rot_axis, -2.0 * alpha);

                    stick_voting_direction = q.rotate(ref_dir);
                    theta = alpha;

                    // LOG(INFO) << "The stick voting direction is calculated successfully.";
                }
                else
                {
                    stick_voting_direction.set(0, 0, 0);
                    theta = 0;
                    // LOG(INFO) << "The angle is larger than the threshold. Thus, the stick voting direction is set to zero.";
                }
                return true;
            }
            else
            {
                // LOG(ERROR) << "The tensor of the voter is unavailable.";
                return false;
            }
        }

        /**
         * @brief This function calculates the stick voting decay of a tensor token from a voter to votee.
         *
         * @param votee The tensor token to receive the vote.
         * @param theta The angle between the stick voting direction and the direction of v = votee - voter.
         * @param sigma The radius used for neighbourhood searching and tensor voting.
         * @param decay The calculated decay of the voting.
         * @return true If the stick voting direction and decay are calculated successfully.
         * @return false If the stick voting direction and decay are not calculated.
         */
        bool calc_stick_voting_decay(const TensorToken<FT> &votee,
                                     FT theta, FT sigma, FT &decay)
        {
            if (this->m_voter_available)
            {
                FT const_c{0.0};

                FT arc_length{0.0};
                FT curvature{0.0};

                // If the angle is zero, the arc length is the distance between the voter and the votee.
                if (theta == 0)
                {
                    arc_length = (votee - this->get_position()).length();
                    curvature = 0;
                }
                else
                {
                    arc_length = (theta * (votee - this->get_position()).length() / std::sin(theta));
                    curvature = 2.0 * std::sin(theta) / (votee - this->get_position()).length();
                }
                // The coefficient to balance the weight of the arc length and the curvature.
                const_c = 1.0 * arc_length;

                // The decay is calculated by the formula:
                decay = std::exp(-(std::pow(arc_length, 2) / std::pow(sigma, 2) + const_c * std::pow(curvature, 2)));

                // LOG(INFO) << "The stick voting decay is calculated successfully.";
                return true;
            }
            else
            {
                // LOG(ERROR) << "The tensor of the voter is unavailable thus the decay is not calculated.";
                return false;
            }
        }

    private:
    public:
    }; //!- template class TensorToken;

} //!- namespace mm.

#endif