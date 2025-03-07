#ifndef MM_ALGO_TENSOR_VOTING_H
#define MM_ALGO_TENSOR_VOTING_H

// System
#include <cmath>
#include <vector>

#include <omp.h>
// Local
#include <miemie/basic/matrix.h>
#include <miemie/basic/quat.h>
#include <miemie/tools/kdtree/kdtree_search_eth.h>
#include <miemie/tools/kdtree/kdtree_search_nanoflann.h>

#include <miemie/core/point_cloud.h>
#include <miemie/fileio/point_cloud_io.h>

#include <miemie/util/logging.h>

#include "tensor_token.hpp"

namespace mm
{

    template <typename FT>
    class TensorVoting
    {
    public:
        /// @brief The associated point cloud;
        std::shared_ptr<PointCloud> m_cloud;

        /// @brief The points used for tensor voting.
        std::vector<TensorToken<FT>> m_tensor_tokens;

        /// @brief The radius used for neighbourhood searching and tensor voting.
        FT m_sigma;

        /// @brief The minimum number of neighbours required for tensor voting.
        size_t m_minimum_number_of_neighbours;

        /// @brief Mark if the point cloud is initialized.
        bool m_is_point_cloud_initialized;

        /// @brief Mark if the tensor tokens are calculated.
        bool m_is_tensor_token_calculated;

        /// @brief Mark if the KdTree is constructed.
        bool m_is_kdtree_constructed;

        /// @brief Mark if the sigma is initialized.
        bool m_is_sigma_initialized;

        /// @brief The points used for tensor voting.
        std::vector<Vec3f> m_points;
        // std::vector<size_t> m_indices;
        std::vector<std::pair<Vec3f, size_t>> m_points_indices;

    private:
        /// @brief The KdTree used for neighbourhood searching.
        std::shared_ptr<KdTreeSearch> m_kdtree;

    private:
        class UnionFind
        {
        public:
            std::vector<size_t> parent;
            UnionFind(size_t n) : parent(n)
            {
                for (size_t i = 0; i < n; i++)
                    parent[i] = i;
            }
            size_t find(size_t x)
            {
                if (parent[x] != x)
                    parent[x] = find(parent[x]);
                return parent[x];
            }
            void unite(size_t x, size_t y)
            {
                size_t rootX = find(x);
                size_t rootY = find(y);
                if (rootX != rootY)
                    parent[rootY] = rootX;
            }
        };

    public:
        /**
         * @brief Construct a new Tensor Voting object
         *
         */
        TensorVoting()
            : m_cloud(nullptr),
              m_kdtree(nullptr),
              m_sigma(FT(0)),
              m_minimum_number_of_neighbours(0),
              m_is_point_cloud_initialized(false),
              m_is_tensor_token_calculated(false),
              m_is_kdtree_constructed(false),
              m_is_sigma_initialized(false)
        {
        }

        /**
         * @brief Construct a new Tensor Voting object
         *
         * @param cloud The associated point cloud.
         * @param sigma The radius used for neighbourhood searching and tensor voting.
         * @param m_minimum_number_of_neighbours The minimum number of neighbours required for calculate tensor tokens.
         */
        TensorVoting(const std::shared_ptr<PointCloud> &cloud,
                     FT sigma,
                     size_t minimum_number_of_neighbours = 3)
            : m_cloud(cloud),
              m_sigma(sigma),
              m_minimum_number_of_neighbours(minimum_number_of_neighbours),
              m_is_point_cloud_initialized(true),
              m_is_tensor_token_calculated(false),
              m_is_kdtree_constructed(false),
              m_is_sigma_initialized(true)
        {
        }

        TensorVoting(const std::vector<Vec3f> &points,
                     FT sigma,
                     size_t minimum_number_of_neighbours = 3)
            : m_points(points),
              m_sigma(sigma),
              m_minimum_number_of_neighbours(minimum_number_of_neighbours),
              m_is_point_cloud_initialized(true),
              m_is_tensor_token_calculated(false),
              m_is_kdtree_constructed(false),
              m_is_sigma_initialized(true)
        {
        }

        TensorVoting(const std::shared_ptr<PointCloud> &cloud,
                     std::vector<Vec3f> &points,
                     FT sigma, size_t minimum_number_of_neighbours = 3)
            : m_cloud(cloud),
              m_points(points),
              m_sigma(sigma),
              m_minimum_number_of_neighbours(minimum_number_of_neighbours),
              m_is_point_cloud_initialized(true),
              m_is_tensor_token_calculated(false),
              m_is_kdtree_constructed(false),
              m_is_sigma_initialized(true)
        {
        }

        /**
         * @brief Destroy the Tensor Voting object
         *
         */
        ~TensorVoting() = default;

    public:
        /**
         * @brief Encodes the tensor tokens of the input point cloud data.
         *
         * @return true If the tensor tokens are calculated successfully.
         * @return false If the tensor tokens are not calculated.
         */
        bool encode_tensor_tokens()
        {
            // Calculate the tensor tokens.
            if (this->calculate_tensor_tokens())
            {
                return true;
            }
            else
            {
                // LOG(ERROR) << "The tensor tokens are not calculated.";
                return false;
            }
        }

        bool encode_tensor_tokens_new(const float radius, const float size = 10.0)
        {
            if (!this->m_is_point_cloud_initialized)
            {
                // LOG(ERROR) << "The point cloud is not initialized.";
                return false;
            }
            if (!this->m_is_sigma_initialized)
            {
                // LOG(ERROR) << "The sigma is not initialized.";
                return false;
            }
            if (this->m_is_tensor_token_calculated)
            {
                // LOG(INFO) << "The tensor tokens are already calculated.";
                return true;
            }
            if (!this->m_is_kdtree_constructed)
            {
                this->m_kdtree = std::make_shared<KdTreeSearch_ETH>(this->m_points);
                this->m_is_kdtree_constructed = true;
            }

            for (size_t i = 0; i < this->m_points.size(); ++i)
            {
                this->m_points_indices.push_back(std::make_pair(this->m_points[i], i));
            }

            this->m_tensor_tokens.clear();
            std::vector<TensorToken<FT>>().swap(this->m_tensor_tokens);
            this->m_tensor_tokens.reserve(this->m_points.size());

            std::vector<std::vector<size_t>> clusters = this->cluster_points(this->m_points, 0.2);

            for (const auto &cluster : clusters)
            {
                std::pair<Vec3f, Vec3f> bounding_box = this->compute_bounding_box(this->m_points, cluster);
                const Vec3f &min_coord = bounding_box.first;
                const Vec3f &max_coord = bounding_box.second;

                double dx = max_coord.get_x() - min_coord.get_x();
                double dy = max_coord.get_y() - min_coord.get_y();
                double dz = max_coord.get_z() - min_coord.get_z();

                if (dx < size && dy < size && dz < size)
                {
                    continue;
                }

                for (const auto &idx : cluster)
                {
                    TensorToken<FT> token;
                    token.set(this->m_points[idx]);

                    std::vector<int> neighbours;
                    std::vector<float> sqr_distances;

                    this->m_kdtree->find_points_in_range(this->m_points[idx],
                                                         radius * radius,
                                                         neighbours,
                                                         sqr_distances);
                    if (neighbours.size() <= this->m_minimum_number_of_neighbours)
                    {
                        token.set_outlier(true);
                        token.set_num_neighbours(neighbours.size());
                        token.set_voter(Tensor<FT>(), false);
                        this->m_tensor_tokens.push_back(token);
                        continue;
                    }

                    token.set_num_neighbours(neighbours.size());
                    Matrix<FT> cov_matrix = this->calculate_covariance_matrix(neighbours);

                    Tensor<FT> curr_tensor(cov_matrix(0, 0),
                                           cov_matrix(1, 1),
                                           cov_matrix(2, 2),
                                           cov_matrix(0, 1),
                                           cov_matrix(0, 2),
                                           cov_matrix(1, 2));

                    token.set_voter(curr_tensor, true);

                    this->m_tensor_tokens[this->m_points_indices[idx].second] = token;
                }
            }

            this->m_is_tensor_token_calculated = true;

            return true;
        }

        bool encode_tensor_tokens(const float radius, const float ratio, const float size = 10.0)
        {
            if (!this->m_is_point_cloud_initialized)
            {
                // LOG(ERROR) << "The point cloud is not initialized.";
                return false;
            }
            if (!this->m_is_sigma_initialized)
            {
                // LOG(ERROR) << "The sigma is not initialized.";
                return false;
            }
            if (this->m_is_tensor_token_calculated)
            {
                // LOG(INFO) << "The tensor tokens are already calculated.";
                return true;
            }
            if (!this->m_is_kdtree_constructed)
            {
                this->m_kdtree = std::make_shared<KdTreeSearch_ETH>(this->m_points);
                this->m_is_kdtree_constructed = true;
                std::cout << "KdTree is constructed." << std::endl;
            }

            for (size_t i = 0; i < this->m_points.size(); ++i)
            {
                this->m_points_indices.push_back(std::make_pair(this->m_points[i], i));
            }

            this->m_tensor_tokens.clear();
            std::vector<TensorToken<FT>>().swap(this->m_tensor_tokens);
            this->m_tensor_tokens.resize(this->m_points.size());
            std::cout << "Tensor tokens size: " << this->m_tensor_tokens.size() << std::endl;

            std::vector<std::vector<size_t>> clusters = this->cluster_points(this->m_points, 0.2);

            for (const auto &cluster : clusters)
            {
                std::pair<Vec3f, Vec3f> bounding_box = this->compute_bounding_box(this->m_points, cluster);
                const Vec3f &min_coord = bounding_box.first;
                const Vec3f &max_coord = bounding_box.second;

                double dx = max_coord.get_x() - min_coord.get_x();
                double dy = max_coord.get_y() - min_coord.get_y();
                double dz = max_coord.get_z() - min_coord.get_z();

                float curr_ratio = 0.0;
                if (dx > dy)
                {
                    curr_ratio = dy / dx;
                }
                else
                {
                    curr_ratio = dx / dy;
                }

                if (dx < size && dy < size && dz < size && curr_ratio < ratio)
                {
                    for (const auto &idx : cluster)
                    {
                        TensorToken<FT> token;
                        token.set(this->m_points[idx]);
                        token.set_outlier(true);
                        token.set_num_neighbours(0);
                        token.set_voter(Tensor<FT>(), false);
                        // this->m_tensor_tokens.push_back(token);
                        this->m_tensor_tokens[idx] = token;
                    }
                }
                else
                {
                    for (const auto &idx : cluster)
                    {
                        TensorToken<FT> token;
                        token.set(this->m_points[idx]);

                        std::vector<int> neighbours;
                        std::vector<float> sqr_distances;

                        this->m_kdtree->find_points_in_range(this->m_points[idx],
                                                             radius * radius,
                                                             neighbours,
                                                             sqr_distances);

                        if (neighbours.size() <= this->m_minimum_number_of_neighbours)
                        {
                            token.set(this->m_points[idx]);
                            token.set_outlier(true);
                            token.set_num_neighbours(neighbours.size());
                            token.set_voter(Tensor<FT>(), false);
                            // this->m_tensor_tokens.push_back(token);
                            this->m_tensor_tokens[idx] = token;
                        }
                        else
                        {
                            token.set_num_neighbours(neighbours.size());
                            Matrix<FT> cov_matrix = this->calculate_covariance_matrix(neighbours);

                            Tensor<FT> curr_tensor(cov_matrix(0, 0),
                                                   cov_matrix(1, 1),
                                                   cov_matrix(2, 2),
                                                   cov_matrix(0, 1),
                                                   cov_matrix(0, 2),
                                                   cov_matrix(1, 2));

                            token.set_outlier(false);
                            token.set(this->m_points[idx]);
                            token.set_num_neighbours(neighbours.size());
                            // this->m_tensor_tokens.push_back(token);
                            token.set_voter(curr_tensor, true);
                            this->m_tensor_tokens[idx] = token;
                        }
                    }
                }
            }

            this->m_is_tensor_token_calculated = true;
            return true;
        }

        /**
         * @brief  Encodes the tensor tokens of the input point cloud data.
         *
         * @param mode  The mode to choose.
         * @return true  If the tensor tokens are calculated successfully.
         * @return false  If the tensor tokens are not calculated.
         */
        bool encode_tensor_tokens(size_t mode)
        {
            if (this->calculate_tensor_tokens(mode))
            {
                return true;
            }
            else
            {
                LOG(ERROR) << "The tensor tokens are not calculated.";
                return false;
            }
        }

        /**
         * @brief Perform the sparse stick tensor voting.
         *
         * @return true If the sparse stick tensor voting is performed successfully.
         * @return false If the sparse stick tensor voting is not performed.
         */
        bool perform_sparse_stick_voting()
        {
            return this->sparse_stick_tensor_voting();
        }

    private:
        /**
         * @brief Build the kd  tree for the point cloud.
         *
         * @return true If the kd tree is built successfully.
         * @return false If the kd tree is not built.
         */
        bool build_kd_tree()
        {
            if (!this->m_is_point_cloud_initialized)
            {
                // LOG(ERROR) << "The point cloud is not initialized.";
                return false;
            }

            if (this->m_is_kdtree_constructed)
            {
                // LOG(WARNING) << "The KdTree is already constructed.";
                return true;
            }

            this->m_kdtree = std::make_shared<KdTreeSearch_ETH>(this->m_cloud.get());
            this->m_is_kdtree_constructed = true;

            return true;
        }
        /**
         * @brief Calculate the tensor tokens within the given sigma (radius) neighbourhood.
         *
         * @return true If the tensor tokens are calculated successfully.
         * @return false If the tensor tokens are not calculated.
         */
        bool calculate_tensor_tokens()
        {

            if (!this->m_is_point_cloud_initialized)
            {
                // LOG(ERROR) << "The point cloud is not initialized.";
                return false;
            }
            if (!this->m_is_sigma_initialized)
            {
                // LOG(ERROR) << "The sigma is not initialized.";
                return false;
            }
            if (this->m_is_tensor_token_calculated)
            {
                // LOG(INFO) << "The tensor tokens are already calculated.";
                return true;
            }
            if (!this->m_is_kdtree_constructed)
            {
                this->m_kdtree = std::make_shared<KdTreeSearch_ETH>(this->m_cloud.get());
                this->m_is_kdtree_constructed = true;
            }

            this->m_tensor_tokens.clear();
            std::vector<TensorToken<FT>>().swap(this->m_tensor_tokens);
            this->m_tensor_tokens.reserve(this->m_cloud->n_vertices());

            for (size_t i = 0; i < this->m_cloud->n_vertices(); ++i)
            {
                // Set the position of the tensor token to the current point.
                TensorToken<FT> token;
                token.set(this->m_cloud->points()[i]);

                std::vector<int> neighbours;
                std::vector<float> sqr_distances;

                this->m_kdtree->find_points_in_range(this->m_cloud->points()[i],
                                                     this->m_sigma * this->m_sigma,
                                                     neighbours,
                                                     sqr_distances);
                // If the number of neighbours is less than or equal to the 'minimum_number_of_points'
                // mark the point as an outlier. Also, the tensor token is set unavailable. Just skip and continue.
                if (neighbours.size() <= this->m_minimum_number_of_neighbours)
                {
                    token.set_outlier(true);
                    token.set_num_neighbours(neighbours.size());
                    token.set_voter(Tensor<FT>(), false);
                    this->m_tensor_tokens.push_back(token);
                    continue;
                }

                token.set_num_neighbours(neighbours.size());
                // Calculate the covariance matrix of the neighbours.
                Matrix<FT> cov_matrix = this->calculate_covariance_matrix(neighbours);

                // Initialize the tensor token with the covariance matrix.
                Tensor<FT> curr_tensor(cov_matrix(0, 0),
                                       cov_matrix(1, 1),
                                       cov_matrix(2, 2),
                                       cov_matrix(0, 1),
                                       cov_matrix(0, 2),
                                       cov_matrix(1, 2));

                // Set the tensor to the tensor token (voter). Also split the tensor.
                token.set_voter(curr_tensor, true);

                // Push the tensor token to the tensor token container.
                this->m_tensor_tokens.push_back(token);
            }

            // Mark the tensor tokens are calculated.
            this->m_is_tensor_token_calculated = true;

            return true;
        }

        bool calculate_tensor_tokens(size_t mode)
        {
            if (!this->m_is_point_cloud_initialized)
            {
                // LOG(ERROR) << "The point cloud is not initialized.";
                return false;
            }

            if (!this->m_is_sigma_initialized || this->m_sigma <= 0)
            {
                // LOG(ERROR) << "The sigma is either not set or invalid value (<=0).";
                return false;
            }

            if (this->m_is_tensor_token_calculated)
            {
                // LOG(INFO) << "The tensor tokens are already calculated.";
                return true;
            }

            if (!this->m_is_kdtree_constructed)
            {
                this->m_kdtree = std::make_shared<KdTreeSearch_ETH>(this->m_cloud.get());
                this->m_is_kdtree_constructed = true;
            }

            this->m_tensor_tokens.clear();
            std::vector<TensorToken<FT>>().swap(this->m_tensor_tokens);
            this->m_tensor_tokens.reserve(this->m_cloud->n_vertices());

            for (size_t i = 0; i < this->m_cloud->n_vertices(); ++i)
            {
                // Set the position of the tensor token to the current point.
                TensorToken<FT> token;
                token.set(this->m_cloud->points()[i]);

                // Find the neighbours within the sigma (radius) neighbourhood.
                std::vector<int> neighbors;
                std::vector<float> sqr_distances;
                this->m_kdtree->find_points_in_range(this->m_cloud->points()[i],
                                                     this->m_sigma * this->m_sigma,
                                                     neighbors,
                                                     sqr_distances);
                // If the number of neighbours is less than or  equalt to the 'minmum_number_of_neighbours',
                // mark the token as outlier. Also, the tensor token is set unavailable. Just skip and continue.
                if (neighbors.size() <= m_minimum_number_of_neighbours)
                {
                    token.set_outlier(true);
                    token.set_voter(Tensor<FT>(), false);
                    token.set_num_neighbours(neighbors.size());

                    this->m_tensor_tokens.push_back(token);
                    continue;
                }

                // calcualte the covariance matrix of the neighbours.
                Matrix<FT> cov_matrix;
                if (mode == 1)
                {
                    cov_matrix = this->calculate_covariance_matrix(neighbors, this->m_cloud->points()[i]);
                }
                else if (mode == 2)
                {
                    cov_matrix = this->calculate_covariance_matrix(neighbors);
                }
                else
                {
                    // LOG(ERROR) << "The mode is not supported.";
                    return false;
                }

                Tensor<FT> curr_tensor(cov_matrix(0, 0),
                                       cov_matrix(1, 1),
                                       cov_matrix(2, 2),
                                       cov_matrix(0, 1),
                                       cov_matrix(0, 2),
                                       cov_matrix(1, 2));
                // Set the tensor to the tensor token. Also split the tensor.
                token.set_voter(curr_tensor, true);

                // Push the tensor token to the tensor token container.
                this->m_tensor_tokens.push_back(token);
            }

            this->m_is_tensor_token_calculated = true;

            return true;
        }

        /**
         * @brief The function calculates the covariance matrix of the neighbours of a query point.
         *
         * @note The covariance matrix is calculated w.r.t the query point.
         *
         * @param neighbours The indices of the neighbours.
         * @param query_point The query point.
         * @return Matrix<FT> The obtained covariance matrix.
         */
        Matrix<FT> calculate_covariance_matrix(const std::vector<int> &neighbours, const Vec3f &query_point)
        {
            Matrix<FT> cov_matrix(3, 3);
            cov_matrix.load_zero();

            Matrix<FT> coeffi_matrix(neighbours.size(), 3);
            coeffi_matrix.load_zero();

            for (size_t i = 1; i < neighbours.size(); ++i)
            {
                coeffi_matrix(i, 0) = m_cloud->points()[neighbours[i]].get_x() - query_point.get_x();
                coeffi_matrix(i, 1) = m_cloud->points()[neighbours[i]].get_y() - query_point.get_y();
                coeffi_matrix(i, 2) = m_cloud->points()[neighbours[i]].get_z() - query_point.get_z();
            }

            cov_matrix = transpose(coeffi_matrix) * coeffi_matrix;

            return cov_matrix;
        }

        /**
         * @brief Calculate the tensor tokens within the given sigma (radius) neighbourhood.
         *
         * @param neighbours The neighbours of the current point.
         * @return Matrix<FT> The covariance matrix of the neighbours.
         */
        Matrix<FT> calculate_covariance_matrix(const std::vector<int> &neighbours)
        {
            Matrix<FT> cov_matrix(3, 3);
            cov_matrix.load_zero();

            Vec<3, FT> centroid(FT(0), FT(0), FT(0));
            for (const auto &neighbour : neighbours)
            {
                centroid += m_cloud->points()[neighbour];
            }
            centroid /= neighbours.size();

            Matrix<FT> coeffi_matrix(neighbours.size(), 3);
            coeffi_matrix.load_zero();

            for (size_t i = 0; i < neighbours.size(); ++i)
            {
                coeffi_matrix(i, 0) = m_cloud->points()[neighbours[i]].get_x() - centroid.get_x();
                coeffi_matrix(i, 1) = m_cloud->points()[neighbours[i]].get_y() - centroid.get_y();
                coeffi_matrix(i, 2) = m_cloud->points()[neighbours[i]].get_z() - centroid.get_z();
            }

            cov_matrix = transpose(coeffi_matrix) * coeffi_matrix;

            cov_matrix /= neighbours.size();

            return cov_matrix;
        }

        std::vector<std::vector<size_t>> cluster_points(std::vector<Vec3f> points, const float radius)
        {
            std::vector<std::vector<size_t>> clusters;
            size_t N = points.size();

            if (N == 0)
            {
                return clusters;
            }

            auto kdtree = std::make_shared<KdTreeSearch_NanoFLANN>(points);

            UnionFind uf(N);
            double search_radius = static_cast<double>(radius * radius);

            for (size_t i = 0; i < N; ++i)
            {
                std::vector<int> neighbours;
                kdtree->find_points_in_range(points[i], search_radius, neighbours);

                for (const auto &neighbour : neighbours)
                {
                    uf.unite(i, static_cast<size_t>(neighbour));
                }
            }

            std::unordered_map<size_t, std::vector<size_t>> clusters_map;
            for (size_t i = 0; i < N; ++i)
            {
                clusters_map[uf.find(i)].push_back(i);
            }

            clusters.reserve(clusters_map.size());
            for (auto &entry : clusters_map)
            {
                clusters.push_back(entry.second);
            }

            return clusters;
        }

        std::pair<Vec3f, Vec3f> compute_bounding_box(const std::vector<Vec3f> &points,
                                                     const std::vector<size_t> &cluster)
        {
            Vec3f min_coord(std::numeric_limits<float>::max());
            Vec3f max_coord(std::numeric_limits<float>::lowest());

            for (const auto &idx : cluster)
            {
                min_coord = std::min(min_coord, points[idx]);
                max_coord = std::max(max_coord, points[idx]);
            }

            return std::make_pair(min_coord, max_coord);
        }

        /**
         * @brief This function performs the sparse stick tensor voting.
         *
         * @note The obtained tensor at each votee is the sum of the voting tensors from all the voters.
         *       However, the obtained tensor is not yet splited into stick, plate, and sphere tensors.
         *
         * @return true If the sparse stick tensor voting is performed successfully.
         * @return false If the sparse stick tensor voting is not performed.
         */
        bool sparse_stick_tensor_voting_old()
        {
            if (!this->m_is_tensor_token_calculated)
            {
                // LOG(ERROR) << "The tensor tokens are not calculated.";
                return false;
            }

            // #pragma omp parallel for num_threads(16) schedule(dynamic) shared(neighbours)
            for (size_t i = 0; i < this->m_tensor_tokens.size(); ++i)
            {
                // Skip the outlier.
                if (this->m_tensor_tokens[i].is_outlier())
                {
                    continue;
                }
                if (this->m_tensor_tokens[i].get_num_neighbours() <= 80)
                {
                    continue;
                }
                if (this->m_tensor_tokens[i].get_voter().get_stickness() <= 0.7 ||
                    this->m_tensor_tokens[i].get_voter().get_stickness() >= 0.95)
                {
                    continue;
                }

                // The voter tensor token.
                TensorToken<FT> voter = this->m_tensor_tokens[i];

                Vec<3, FT> voter_position = voter.get_position();
                FT stickness = voter.get_voter().get_stickness();
                Vec<3, FT> stick_voting_direction = voter.get_voter().get_e1();

                // This is the voting radius, which can be tested with different parameters.
                FT voting_radius = 1.5 * this->m_sigma;

                std::vector<int> neighbours;
                std::vector<float> sqr_distances;
                this->m_kdtree->find_points_in_range(voter_position,
                                                     voting_radius * voting_radius,
                                                     neighbours,
                                                     sqr_distances);
                /*for (auto idx : neighbours)
                {
                    // The three variables that has to be calculated for each votee in the neighbourhood.
                    Vec<3, FT> calculated_voting_dir_at_votee;
                    FT decay = FT(0);
                    FT theta = FT(0);

                    this->m_tensor_tokens[i].calc_stick_voting_direction(this->m_tensor_tokens[idx],
                                                                         calculated_voting_dir_at_votee,
                                                                         theta, M_PI / 4.0);
                    this->m_tensor_tokens[i].calc_stick_voting_decay(this->m_tensor_tokens[idx],
                                                                     theta,
                                                                     this->m_sigma, decay);

                    // Create a tensor using the calculated voting direction as votee.
                    Tensor<FT> voting_stick_tensor_at_votee = Tensor<FT>(calculated_voting_dir_at_votee);
                    // Multiply the tensor with the decay and the stickness of the voter.
                    voting_stick_tensor_at_votee *= (decay * stickness);

                    this->m_tensor_tokens[idx].m_votee.get_base_object() += voting_stick_tensor_at_votee;
                }*/

                for (size_t j = 1; j < neighbours.size(); ++j)
                {
                    size_t idx = neighbours[j];

                    Vec<3, FT> calculated_voting_dir_at_votee;
                    FT decay = FT(0);
                    FT theta = FT(0);

                    this->m_tensor_tokens[i].calc_stick_voting_direction(this->m_tensor_tokens[idx],
                                                                         calculated_voting_dir_at_votee,
                                                                         theta, M_PI / 4.0);
                    this->m_tensor_tokens[i].calc_stick_voting_decay(this->m_tensor_tokens[idx],
                                                                     theta,
                                                                     this->m_sigma, decay);

                    Tensor<FT> voting_stick_tensor_at_votee = Tensor<FT>(calculated_voting_dir_at_votee);
                    voting_stick_tensor_at_votee *= (decay * stickness);

                    this->m_tensor_tokens[idx].m_votee.get_base_object() += voting_stick_tensor_at_votee;
                }
            }
            return true;
        }

        bool sparse_stick_tensor_voting()
        {
            if (!this->m_is_tensor_token_calculated)
            {
                // LOG(ERROR) << "The tensor tokens are not calculated.";
                return false;
            }

            for (size_t i = 0; i < this->m_tensor_tokens.size(); ++i)
            {
                if (this->m_tensor_tokens[i].is_outlier())
                {
                    continue;
                }
                /*if (this->m_tensor_tokens[i].get_voter().get_stickness() <= 0.7 ||
                    this->m_tensor_tokens[i].get_voter().get_stickness() >= 0.95)
                {
                    continue;
                }*/

                // The voter tensor token.
                TensorToken<FT> voter = this->m_tensor_tokens[i];

                Vec<3, FT> voter_position = voter.get_position();
                FT stickness = voter.get_voter().get_stickness();
                Vec<3, FT> stick_voting_direction = voter.get_voter().get_e1();

                // This is the voting radius, which can be tested with different parameters.
                FT voting_radius = this->m_sigma;

                std::vector<int> neighbours;
                std::vector<float> sqr_distances;
                this->m_kdtree->find_points_in_range(voter_position,
                                                     voting_radius * voting_radius,
                                                     neighbours,
                                                     sqr_distances);

                for (size_t j = 0; j < neighbours.size(); ++j)
                {
                    size_t idx = neighbours[j];
                    Vec<3, FT> calculated_voting_dir_at_votee;
                    FT decay = FT(0);
                    FT theta = FT(0);

                    this->m_tensor_tokens[i].calc_stick_voting_direction(this->m_tensor_tokens[idx],
                                                                         calculated_voting_dir_at_votee,
                                                                         theta, M_PI / 4.0);
                    this->m_tensor_tokens[i].calc_stick_voting_decay(this->m_tensor_tokens[idx],
                                                                     theta,
                                                                     this->m_sigma, decay);

                    Tensor<FT> voting_stick_tensor_at_votee = Tensor<FT>(calculated_voting_dir_at_votee);
                    voting_stick_tensor_at_votee *= (decay * stickness);

                    this->m_tensor_tokens[idx].m_votee.get_base_object() += voting_stick_tensor_at_votee;
                }
            }
            return true;
        }

    }; //!- template class TensorVoting.
    // I did not find the closing brace for the namespace mm.
} //!- namespace mm.

#endif //!- MM_ALGO_TENSOR_VOTING_H
