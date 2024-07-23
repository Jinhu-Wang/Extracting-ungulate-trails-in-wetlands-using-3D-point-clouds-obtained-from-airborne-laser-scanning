
// Internal Includes
#include "MarchingSquares.h"
#include "Edge.h"
#include "Node.h"

// Standard includes
#include <iostream>
#include <map>
#include <utility> // For std::pair

namespace ms
{

    /**
     * Converts the binary array to a single int
     *
     * @param arr Array containing the bool values
     * @param size The size if the array (see array decay)
     *
     * @return Integer value of the given binary number, or 0 if empty
     */
    inline size_t ToInt(const bool arr[], const size_t size)
    {
        size_t result = 0;

        for (size_t i = 0; i < size; ++i)
            result |= static_cast<size_t>(arr[i]) << (size - i - 1);

        return result;
    }

    inline Point2D LinearInterpolate(const Point2D &origin,
                                     const Point2D &&target,
                                     const Point2D &values,
                                     const double iso_value)
    {
        const auto t = (iso_value - values[0]) / (values[1] - values[0]);

        return {
            (1 - t) * origin[0] + t * target[0],
            (1 - t) * origin[1] + t * target[1]};
    }

    /**
     * Constructor of the marching squares class
     *
     * @param function The function for which marching squares needs to be generated
     * @param x_limits The lower and upper bounds in the x direction
     * @param y_limits The lower and upper bounds in the y direction
     * @param resolution The number of cells in the x and y direction
     */
    MarchingSquares::MarchingSquares(Function function,
                                     const Limits &x_limits,
                                     const Limits &y_limits,
                                     const Resolution &resolution)
        : function_(std::move(function)),
          x_limits_(x_limits),
          y_limits_(y_limits),
          resolution_(verify_resolution(resolution)),
          dx_((x_limits[1] - x_limits[0]) / resolution_[0]),
          dy_((y_limits[1] - y_limits[0]) / resolution_[1]),
          cur_x_(x_limits[0]),
          cur_y_(y_limits[0]),
          x_major_(resolution[0] > resolution[1]),
          m_nKDTree(nullptr),
          m_nIsKDTreeBuilt(false)
    {
        nx1_ = resolution[!x_major_];
        nx2_ = resolution[x_major_];
    }

    MarchingSquares::MarchingSquares(PointCloud *cloud,
                                     const Limits &x_limits,
                                     const Limits &y_limits,
                                     const Resolution &resolution)
        : x_limits_(x_limits),
          y_limits_(y_limits),
          resolution_(verify_resolution(resolution)),
          dx_((x_limits[1] - x_limits[0]) / resolution_[0]),
          dy_((y_limits[1] - y_limits[0]) / resolution_[1]),
          cur_x_(x_limits[0]),
          cur_y_(y_limits[0]),
          x_major_(resolution[0] > resolution[1]),
          m_nKDTree(nullptr),
          m_nIsKDTreeBuilt(false)
    {
        nx1_ = resolution[!x_major_];
        nx2_ = resolution[x_major_];

        std::map<std::pair<double, double>, double> myMap;

        std::vector<double> x_coords;
        std::vector<double> y_coords;
        for (size_t i = 0; i < cloud->m_PtsNum; ++i)
        {
            x_coords.push_back(cloud->m_Points[i].x);
            y_coords.push_back(cloud->m_Points[i].y);
            myMap[std::make_pair(cloud->m_Points[i].x,
                                 cloud->m_Points[i].y)] = cloud->m_Points[i].z;
        }

        std::sort(x_coords.begin(), x_coords.end());
        auto lastX = std::unique(x_coords.begin(), x_coords.end());
        x_coords.erase(lastX, x_coords.end());

        std::sort(y_coords.begin(), y_coords.end());
        auto lastY = std::unique(y_coords.begin(), y_coords.end());
        y_coords.erase(lastY, y_coords.end());

        // Generate the grid
        for (size_t i = 0; i < x_coords.size() - 1; ++i)
        {
            for (size_t j = 0; j < y_coords.size() - 1; ++j)
            {
                std::pair<double, double> key0 = std::make_pair(x_coords[i], y_coords[j]);
                std::pair<double, double> key1 = std::make_pair(x_coords[i + 1], y_coords[j]);
                std::pair<double, double> key2 = std::make_pair(x_coords[i + 1], y_coords[j + 1]);
                std::pair<double, double> key3 = std::make_pair(x_coords[i], y_coords[j + 1]);

                double z0, z1, z2, z3;
                if (myMap.find(key0) != myMap.end() &&
                    myMap.find(key1) != myMap.end() &&
                    myMap.find(key2) != myMap.end() &&
                    myMap.find(key3) != myMap.end())
                {
                    z0 = myMap[key0];
                    z1 = myMap[key1];
                    z2 = myMap[key2];
                    z3 = myMap[key3];

                    // Calculate the node coordinates
                    std::array<ms::Node, 4> nodes = {{
                        {z0, x_coords[i], y_coords[j], 0.0},         // Node 1
                        {z1, x_coords[i + 1], y_coords[j], 0.0},     // Node 2
                        {z2, x_coords[i + 1], y_coords[j + 1], 0.0}, // Node 3
                        {z3, x_coords[i], y_coords[j + 1], 0.0},     // Node 4
                    }};

                    m_Grids.push_back(nodes);
                }
                else
                {
                    continue;
                }
            }
        }
    }

    Resolution MarchingSquares::verify_resolution(const Resolution &resolution)
    {
        auto new_resolution = resolution;
        if (resolution[0] < 1)
        {
            std::cerr << "MarchingSquares::Constructor: The resolution in x direction must be greater than 0. Automatically setting it to 1\n";
            new_resolution[0] = 1;
        }
        if (resolution[1] < 1)
        {
            std::cerr << "MarchingSquares::Constructor: The resolution in y direction must be greater than 0. Automatically setting it to 1\n";
            new_resolution[1] = 1;
        }

        return new_resolution;
    }

    EdgeVertices MarchingSquares::edge_id_to_nodes(const size_t id) const
    {
        if (id >= total_edges_)
            return {};

        return edge_to_vertices_[id];
    }

    EdgeIndexList MarchingSquares::case_to_edges(const size_t id) const
    {
        if (id >= total_cases_)
            return {};

        return lookup_table_[id];
    }

    void MarchingSquares::increment_minor_axis() const
    {
        if (x_major_)
            cur_y_ += dy_;
        else
            cur_x_ += dx_;
    }

    void MarchingSquares::reset_minor_axis() const
    {
        if (x_major_)
            cur_y_ = y_limits_[0];
        else
            cur_x_ = x_limits_[0];
    }

    void MarchingSquares::increment_major_axis() const
    {
        if (x_major_)
            cur_x_ += dx_;
        else
            cur_y_ += dy_;
    }

    void MarchingSquares::reset_major_axis() const
    {
        if (x_major_)
            cur_x_ = x_limits_[0];
        else
            cur_y_ = y_limits_[0];
    }

    EdgeList MarchingSquares::compute(const double iso_value) const
    {
        EdgeList edge_list;

        // Extract the entries for better readability
        const auto x_lower = x_limits_[0];
        const auto x_upper = x_limits_[1];
        const auto y_lower = y_limits_[0];
        const auto y_upper = y_limits_[1];

        const auto dx = (x_upper - x_lower) / resolution_[0];
        const auto dy = (y_upper - y_lower) / resolution_[1];

        // Go over all the cells
        for (size_t i = 0; i < resolution_[0]; ++i)
        {
            const auto x = x_lower + i * dx;

            for (size_t j = 0; j < resolution_[1]; ++j)
            {
                const auto y = y_lower + j * dy;

                // Calculate the node coordinates
                std::array<Node, 4> nodes = {{
                    {function_(x_lower + i * dx, y_lower + (j + 1) * dy), x, y + dy},            // Node 1
                    {function_(x_lower + (i + 1) * dx, y_lower + (j + 1) * dy), x + dx, y + dy}, // Node 2
                    {function_(x_lower + (i + 1) * dx, y_lower + j * dy), x + dx, y},            // Node 3
                    {function_(x_lower + i * dx, y_lower + j * dy), x, y},                       // Node 4
                }};

                // Compute the function signs based on iso value
                bool func_signs[4];

                for (size_t k = 0; k < 4; ++k)
                {
                    func_signs[k] = (iso_value - nodes[k].Value()) > 0;
                }

                // Compute the corresponding key from the function signs
                const auto key = ToInt(func_signs, 4);

                // Get the edges corresponding to the key
                const auto edges = case_to_edges(key);

                // Compute and add the edge after interpolation
                if (edges.size() == 2)
                {
                    // Get the coordinates of the first edge
                    auto nodes_id = edge_id_to_nodes(edges[0]);
                    const auto origin = Edge(&nodes[nodes_id[0]], &nodes[nodes_id[1]]).GetIsoCoordinates(iso_value);

                    // Get the coordinates of the second edge
                    nodes_id = edge_id_to_nodes(edges[1]);
                    const auto target = Edge(&nodes[nodes_id[0]], &nodes[nodes_id[1]]).GetIsoCoordinates(iso_value);

                    edge_list.push_back({{{origin[0], origin[1]},
                                          {target[0], target[1]}}});
                }
                else if (edges.size() == 4)
                {
                    auto nodes_id = edge_id_to_nodes(edges[0]);
                    auto point1 = Edge(&nodes[nodes_id[0]], &nodes[nodes_id[1]]).GetIsoCoordinates(iso_value);

                    nodes_id = edge_id_to_nodes(edges[1]);
                    const auto point2 = Edge(&nodes[nodes_id[0]], &nodes[nodes_id[1]]).GetIsoCoordinates(iso_value);

                    nodes_id = edge_id_to_nodes(edges[2]);
                    auto point3 = Edge(&nodes[nodes_id[0]], &nodes[nodes_id[1]]).GetIsoCoordinates(iso_value);

                    nodes_id = edge_id_to_nodes(edges[3]);
                    const auto point4 = Edge(&nodes[nodes_id[0]], &nodes[nodes_id[1]]).GetIsoCoordinates(iso_value);

                    // Sort based on the x-component of the array
                    if (point1[0] > point3[0])
                        std::swap(point1, point3);

                    edge_list.push_back({{{point1[0], point1[1]},
                                          {point4[0], point4[1]}}});

                    edge_list.push_back({{{point2[0], point2[1]},
                                          {point3[0], point3[1]}}});
                }
            }
        }

        return edge_list;
    }

    MarchingSquares::MarchingSquares(const Limits &x_limits, const Limits &y_limits,
                                     const Resolution &resolution, const std::vector<dPoint3D> points)
        : x_limits_(x_limits),
          y_limits_(y_limits),
          resolution_(verify_resolution(resolution)),
          dx_((x_limits[1] - x_limits[0]) / resolution_[0]),
          dy_((y_limits[1] - y_limits[0]) / resolution_[1]),
          cur_x_(x_limits[0]),
          cur_y_(y_limits[0]),
          x_major_(resolution[0] > resolution[1]),
          m_nKDTree(nullptr),
          m_nIsKDTreeBuilt(false)
    {

        nx1_ = resolution[!x_major_];
        nx2_ = resolution[x_major_];

        std::map<std::pair<double, double>, double> myMap;

        PointCloud2<double> kdPts;
        kdPts.pts.resize(points.size());
        for (int i = 0; i < points.size(); ++i)
        {
            kdPts.pts[i].x = points[i].x;
            kdPts.pts[i].y = points[i].y;
            // kdPts.pts[i].z = cloud->m_Points[i].z;
            kdPts.pts[i].z = 0.0;
        }

        kdTree *tree = new kdTree(3, kdPts, KDTreeSingleIndexAdaptorParams(10));
        tree->buildIndex();

        // Extract the entries for better readability;
        const auto x_lower = x_limits_[0];
        const auto x_upper = x_limits_[1];
        const auto y_lower = y_limits_[0];
        const auto y_upper = y_limits_[1];

        const auto dx = (x_upper - x_lower) / resolution_[0];
        const auto dy = (y_upper - y_lower) / resolution_[1];

        std::vector<double> x_coords;
        std::vector<double> y_coords;

        int kkkkk = 0;
        for (size_t i = 0; i < resolution_[0]; ++i)
        {
            const auto x = x_lower + i * dx;
            for (size_t j = 0; j < resolution_[1]; ++j)
            {
                const auto y = y_lower + j * dy;
                x_coords.push_back(x);
                y_coords.push_back(y);

                double queryPt[3];
                queryPt[0] = x;
                queryPt[1] = y;
                queryPt[2] = 0.0;

                const size_t ret_num = 1; // number of closest points;
                std::vector<size_t> ret_idx(ret_num);
                std::vector<double> out_dist_sqr(ret_num);
                tree->knnSearch(&queryPt[0], ret_num, &ret_idx[0], &out_dist_sqr[0]);

                double height = -99999.0;
                if (out_dist_sqr[0] < 2.0 * dx * dx)
                {
                    size_t ptID = ret_idx[0];
                    // height = points[ptID].val;
                    // height = points[ptID].val;
                    height = 100.0;
                    kkkkk++;
                }
                myMap[std::make_pair(x, y)] = height;
            }
        }

        std::sort(x_coords.begin(), x_coords.end());
        auto lastX = std::unique(x_coords.begin(), x_coords.end());
        x_coords.erase(lastX, x_coords.end());

        std::sort(y_coords.begin(), y_coords.end());
        auto lastY = std::unique(y_coords.begin(), y_coords.end());
        y_coords.erase(lastY, y_coords.end());

        // Generate the grid
        for (size_t i = 0; i < x_coords.size() - 1; ++i)
        {
            for (size_t j = 0; j < y_coords.size() - 1; ++j)
            {
                std::pair<double, double> key0 = std::make_pair(x_coords[i], y_coords[j]);
                std::pair<double, double> key1 = std::make_pair(x_coords[i + 1], y_coords[j]);
                std::pair<double, double> key2 = std::make_pair(x_coords[i + 1], y_coords[j + 1]);
                std::pair<double, double> key3 = std::make_pair(x_coords[i], y_coords[j + 1]);

                double z0, z1, z2, z3;
                if (myMap.find(key0) != myMap.end() &&
                    myMap.find(key1) != myMap.end() &&
                    myMap.find(key2) != myMap.end() &&
                    myMap.find(key3) != myMap.end())
                {
                    z0 = myMap[key0];
                    z1 = myMap[key1];
                    z2 = myMap[key2];
                    z3 = myMap[key3];

                    // Calculate the node coordinates
                    std::array<ms::Node, 4> nodes = {{
                        {z0, x_coords[i], y_coords[j], 0.0},         // Node 1
                        {z1, x_coords[i + 1], y_coords[j], 0.0},     // Node 2
                        {z2, x_coords[i + 1], y_coords[j + 1], 0.0}, // Node 3
                        {z3, x_coords[i], y_coords[j + 1], 0.0},     // Node 4
                    }};

                    m_Grids.push_back(nodes);
                }
            }
        }
    }

    MarchingSquares::MarchingSquares(const Limits &x_limits, const Limits &y_limits,
                                     const Resolution &resolution, const PointCloud *cloud)
        : x_limits_(x_limits),
          y_limits_(y_limits),
          resolution_(verify_resolution(resolution)),
          dx_((x_limits[1] - x_limits[0]) / resolution_[0]),
          dy_((y_limits[1] - y_limits[0]) / resolution_[1]),
          cur_x_(x_limits[0]),
          cur_y_(y_limits[0]),
          x_major_(resolution[0] > resolution[1]),
          m_nKDTree(nullptr),
          m_nIsKDTreeBuilt(false)
    {
        nx1_ = resolution[!x_major_];
        nx2_ = resolution[x_major_];

        std::map<std::pair<double, double>, double> myMap;

        PointCloud2<double> kdPts;
        kdPts.pts.resize(cloud->m_PtsNum);
        for (int i = 0; i < cloud->m_PtsNum; ++i)
        {
            kdPts.pts[i].x = cloud->m_Points[i].x;
            kdPts.pts[i].y = cloud->m_Points[i].y;
            kdPts.pts[i].z = 0.0;
        }

        kdTree *tree = new kdTree(3, kdPts, KDTreeSingleIndexAdaptorParams(10));
        tree->buildIndex();

        // Extract the entries for better readability;
        const auto x_lower = x_limits_[0];
        const auto x_upper = x_limits_[1];
        const auto y_lower = y_limits_[0];
        const auto y_upper = y_limits_[1];

        const auto dx = (x_upper - x_lower) / resolution_[0];
        const auto dy = (y_upper - y_lower) / resolution_[1];

        std::vector<double> x_coords;
        std::vector<double> y_coords;

        int kkkkk = 0;
        for (size_t i = 0; i < resolution_[0]; ++i)
        {
            const auto x = x_lower + i * dx;
            for (size_t j = 0; j < resolution_[1]; ++j)
            {
                const auto y = y_lower + j * dy;
                x_coords.push_back(x);
                y_coords.push_back(y);

                double queryPt[3];
                queryPt[0] = x;
                queryPt[1] = y;
                queryPt[2] = 0.0;

                const size_t ret_num = 1; // number of closest points;
                std::vector<size_t> ret_idx(ret_num);
                std::vector<double> out_dist_sqr(ret_num);
                tree->knnSearch(&queryPt[0], ret_num, &ret_idx[0], &out_dist_sqr[0]);

                double height = -99999.0;
                if (out_dist_sqr[0] < 2.0 * dx * dx)
                {
                    size_t ptID = ret_idx[0];
                    // height = cloud->m_xyzPoint[ptID].z;
                    // height = cloud->m_xyzPoint[ptID].val;
                    height = 100.0;
                    kkkkk++;
                }
                myMap[std::make_pair(x, y)] = height;
            }
        }

        int kkk = 0;
        std::sort(x_coords.begin(), x_coords.end());
        auto lastX = std::unique(x_coords.begin(), x_coords.end());
        x_coords.erase(lastX, x_coords.end());

        std::sort(y_coords.begin(), y_coords.end());
        auto lastY = std::unique(y_coords.begin(), y_coords.end());
        y_coords.erase(lastY, y_coords.end());
        // Generate the grid
        for (size_t i = 0; i < x_coords.size() - 1; ++i)
        {
            for (size_t j = 0; j < y_coords.size() - 1; ++j)
            {
                std::pair<double, double> key0 = std::make_pair(x_coords[i], y_coords[j]);
                std::pair<double, double> key1 = std::make_pair(x_coords[i + 1], y_coords[j]);
                std::pair<double, double> key2 = std::make_pair(x_coords[i + 1], y_coords[j + 1]);
                std::pair<double, double> key3 = std::make_pair(x_coords[i], y_coords[j + 1]);

                double z0, z1, z2, z3;
                if (myMap.find(key0) != myMap.end() &&
                    myMap.find(key1) != myMap.end() &&
                    myMap.find(key2) != myMap.end() &&
                    myMap.find(key3) != myMap.end())
                {
                    z0 = myMap[key0];
                    z1 = myMap[key1];
                    z2 = myMap[key2];
                    z3 = myMap[key3];

                    // Calculate the node coordinates
                    std::array<ms::Node, 4> nodes = {{
                        {z0, x_coords[i], y_coords[j], 0.0},         // Node 1
                        {z1, x_coords[i + 1], y_coords[j], 0.0},     // Node 2
                        {z2, x_coords[i + 1], y_coords[j + 1], 0.0}, // Node 3
                        {z3, x_coords[i], y_coords[j + 1], 0.0},     // Node 4
                    }};

                    m_Grids.push_back(nodes);
                }
            }
        }

        int kk = 0;
    }

    EdgeList MarchingSquares::compute_ms(const double iso_value) const
    {
        EdgeList edge_list;

        // Extract the entries for better readability
        const auto x_lower = x_limits_[0];
        const auto x_upper = x_limits_[1];
        const auto y_lower = y_limits_[0];
        const auto y_upper = y_limits_[1];
        ;

        const auto dx = (x_upper - x_lower) / resolution_[0];
        const auto dy = (y_upper - y_lower) / resolution_[1];

        for (auto cell : m_Grids)
        {
            std::array<Node, 4> nodes = cell;

            // Compute the function signs based on iso value;
            bool func_signs[4];
            for (size_t k = 0; k < 4; ++k)
            {
                double value = nodes[k].Value();
                func_signs[k] = (iso_value - nodes[k].Value()) > 0;
                // func_signs[k] = (iso_value - nodes[k].Z()) > 0;
            }

            // Compute the corresponding key from the function signs;
            const auto key = ToInt(func_signs, 4);
            if (key != 15)
            {
                int kkk = 0;
            }
            // Get the edges corresponding to the key;
            const auto edges = case_to_edges(key);

            // COmpute and add the edges after interpolation;
            if (edges.size() == 2)
            {
                // Get the coordinates of the first edge
                auto nodes_id = edge_id_to_nodes(edges[0]);
                const auto origin = Edge(&nodes[nodes_id[0]], &nodes[nodes_id[1]]).GetIsoCoordinates(iso_value);

                // Get the coordinates of the second edge
                nodes_id = edge_id_to_nodes(edges[1]);
                const auto target = Edge(&nodes[nodes_id[0]], &nodes[nodes_id[1]]).GetIsoCoordinates(iso_value);

                edge_list.push_back({{{origin[0], origin[1]},
                                      {target[0], target[1]}}});
            }
            else if (edges.size() == 4)
            {
                auto nodes_id = edge_id_to_nodes(edges[0]);
                auto point1 = Edge(&nodes[nodes_id[0]], &nodes[nodes_id[1]]).GetIsoCoordinates(iso_value);

                nodes_id = edge_id_to_nodes(edges[1]);
                const auto point2 = Edge(&nodes[nodes_id[0]], &nodes[nodes_id[1]]).GetIsoCoordinates(iso_value);

                nodes_id = edge_id_to_nodes(edges[2]);
                auto point3 = Edge(&nodes[nodes_id[0]], &nodes[nodes_id[1]]).GetIsoCoordinates(iso_value);

                nodes_id = edge_id_to_nodes(edges[3]);
                const auto point4 = Edge(&nodes[nodes_id[0]], &nodes[nodes_id[1]]).GetIsoCoordinates(iso_value);

                // Sort based on the x-component of the array
                if (point1[0] > point3[0])
                    std::swap(point1, point3);

                edge_list.push_back({{{point1[0], point1[1]},
                                      {point4[0], point4[1]}}});

                edge_list.push_back({{{point2[0], point2[1]},
                                      {point3[0], point3[1]}}});
            }

        } // iterate each cell.
        return edge_list;
    }

    void MarchingSquares::check_vertical_edge(std::vector<double> &arr, const uint32_t index, const double iso_value, Point2D &last_point, std::vector<uint32_t> &index_map) const
    {
        arr[static_cast<size_t>(index) + 1] = function_(cur_x_, cur_y_);

        // If function value signs are opposite
        if ((arr[index] - iso_value) *
                (arr[static_cast<size_t>(index) + 1] - iso_value) <=
            0)
        {
            vertices_.emplace_back(LinearInterpolate(last_point,
                                                     {cur_x_, cur_y_},
                                                     {arr[index], arr[static_cast<size_t>(index) + 1]},
                                                     iso_value));

            index_map[index] = vertex_count_;

            ++vertex_count_;
        }
        last_point = {cur_x_, cur_y_};
    }

    void MarchingSquares::check_horizontal_edge(const Point2D &&values, double iso_value, Point2D &&last_point, uint32_t &index) const
    {
        if ((values[0] - iso_value) *
                (values[1] - iso_value) <=
            0)
        {
            vertices_.emplace_back(LinearInterpolate(last_point,
                                                     {cur_x_, cur_y_},
                                                     values,
                                                     iso_value));
            index = vertex_count_;
            ++vertex_count_;
        }
    }

    void MarchingSquares::reset() const
    {
        reset_major_axis();
        reset_minor_axis();
    }

    Point2D MarchingSquares::get_previous_horizontal_point() const
    {
        if (x_major_)
            return Point2D{cur_x_ - dx_, cur_y_};

        return Point2D{cur_x_, cur_y_ - dy_};
    }

    std::tuple<VerticesList, IndicesList> MarchingSquares::compute_faster(const double iso_value) const
    {
        IndicesList indices;

        // Pre-compute the function values at first column of the minor axis
        std::vector<double> last_col_func(nx2_ + 1);
        std::vector<double> cur_col_func(nx2_ + 1);

        // Map to store the vertices index
        std::vector<uint32_t> last_index_map(nx2_);
        std::vector<uint32_t> cur_index_map(nx2_);

        uint32_t top_index, bottom_index;
        bool pattern[4];
        uint32_t assembled_point_indexes[4];

        // Compute first point of the zeroth column
        last_col_func[0] = function_(cur_x_, cur_y_);
        Point2D last_point = {cur_x_, cur_y_};
        increment_minor_axis();

        // Calculate rest of the entries of the zeroth column
        for (uint32_t j = 0; j < nx2_; ++j)
        {
            check_vertical_edge(last_col_func, j, iso_value, last_point, last_index_map);
            increment_minor_axis();
        }
        reset_minor_axis();
        increment_major_axis();

        // Go over all the cells, iterating primarily over minor axis (smaller boundary, less storage)
        for (uint32_t i = 0; i < nx1_; ++i)
        {
            // Compute the first entry of the current column
            cur_col_func[0] = function_(cur_x_, cur_y_);
            last_point = {cur_x_, cur_y_};

            // Check the first horizontal edge
            check_horizontal_edge(
                {last_col_func[0], cur_col_func[0]},
                iso_value,
                get_previous_horizontal_point(),
                bottom_index);

            increment_minor_axis();

            for (uint32_t j = 0; j < nx2_; ++j)
            {
                check_vertical_edge(cur_col_func, j, iso_value, last_point, cur_index_map);

                // Now we compute the coordinates of the new vertex on the horizontal edge
                check_horizontal_edge(
                    {last_col_func[static_cast<size_t>(j) + 1], cur_col_func[static_cast<size_t>(j) + 1]},
                    iso_value,
                    get_previous_horizontal_point(),
                    top_index);

                assembled_point_indexes[0] = bottom_index;
                assembled_point_indexes[1] = cur_index_map[j];
                assembled_point_indexes[2] = top_index;
                assembled_point_indexes[3] = last_index_map[j];

                // Check the cells case
                pattern[0] = last_col_func[j] - iso_value > 0;
                pattern[1] = cur_col_func[j] - iso_value > 0;
                pattern[2] = cur_col_func[static_cast<size_t>(j) + 1] - iso_value > 0;
                pattern[3] = last_col_func[static_cast<size_t>(j) + 1] - iso_value > 0;

                const auto key = ToInt(pattern, 4);
                const auto intersected_edges = case_to_edges(key);

                if (intersected_edges.size() == 2)
                {
                    indices.emplace_back(std::array<uint32_t, 2>{
                        assembled_point_indexes[intersected_edges[0]],
                        assembled_point_indexes[intersected_edges[1]]});
                }
                // Ambiguous cases
                else if (intersected_edges.size() == 4)
                {
                    // We sort by the x component
                    if (vertices_[assembled_point_indexes[0]][0] >
                        vertices_[assembled_point_indexes[2]][0])
                    {
                        indices.emplace_back(std::array<uint32_t, 2>{
                            assembled_point_indexes[intersected_edges[0]],
                            assembled_point_indexes[intersected_edges[1]]});

                        indices.emplace_back(std::array<uint32_t, 2>{
                            assembled_point_indexes[intersected_edges[2]],
                            assembled_point_indexes[intersected_edges[3]]});
                    }
                    else
                    {
                        indices.emplace_back(std::array<uint32_t, 2>{
                            assembled_point_indexes[intersected_edges[0]],
                            assembled_point_indexes[intersected_edges[3]]});

                        indices.emplace_back(std::array<uint32_t, 2>{
                            assembled_point_indexes[intersected_edges[1]],
                            assembled_point_indexes[intersected_edges[2]]});
                    }
                }

                bottom_index = top_index;
                increment_minor_axis();
            }
            last_index_map.swap(cur_index_map);
            last_col_func.swap(cur_col_func);

            reset_minor_axis();
            increment_major_axis();
        }
        reset();

        vertex_count_ = 0;

        // Vertices_ will implicitly be reset due to std::move
        return {std::move(vertices_), std::move(indices)};
    }
} // namespace marching_squares
