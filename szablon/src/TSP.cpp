#include "TSP.hpp"

#include <algorithm>
#include <stack>
#include <optional>

std::ostream& operator<<(std::ostream& os, const CostMatrix& cm) {
    for (std::size_t r = 0; r < cm.size(); ++r) {
        for (std::size_t c = 0; c < cm.size(); ++c) {
            const auto& elem = cm[r][c];
            os << (is_inf(elem) ? "INF" : std::to_string(elem)) << " ";
        }
        os << "\n";
    }
    os << std::endl;

    return os;
}

long long unsigned int to_numeric(const vertex_t& v) {
    return static_cast<long long unsigned int>(v.row) * 1000000 + v.col;
}

path_t StageState::get_path() {
    path_t sorted_path;

    if (unsorted_path_.size() <= 2) {
        vertex_t first = unsorted_path_[0];
        vertex_t second = unsorted_path_[1];

        // Check matrix condition
        if (matrix_[first.row][second.col] == 0) {
            sorted_path.push_back(to_numeric(first));
            sorted_path.push_back(to_numeric(second));
        } else {
            sorted_path.push_back(to_numeric(second));
            sorted_path.push_back(to_numeric(first));
        }
    } else {
        // If there are more than 2 vertices, push them all as numeric values
        for (const auto& v : unsorted_path_) {
            sorted_path.push_back(to_numeric(v));
        }
    }

    return sorted_path;
}

std::vector<cost_t> CostMatrix::get_min_values_in_rows() const {
    std::vector<cost_t> min_values;
    for (const auto& row : matrix_) {
        if (!row.empty())
            min_values.push_back(*std::min_element(row.begin(), row.end()));
        else
            min_values.push_back(0);
    }

    return min_values;
}

cost_t CostMatrix::reduce_rows() {
    cost_t total_reduction = 0;
    auto min_values = get_min_values_in_rows();

    for (std::size_t i = 0; i < matrix_.size(); ++i) {
        cost_t min_val = min_values[i];

        if (min_val == std::numeric_limits<cost_t>::max() || min_val == 0)
            continue;

        total_reduction += min_val;

        for (auto& cost : matrix_[i]) {
            if (cost != std::numeric_limits<cost_t>::max()) {
                cost -= min_val;
            }
        }
    }

    return total_reduction;
}

std::vector<cost_t> CostMatrix::get_min_values_in_cols() const {

    size_t rows = matrix_.size();
    size_t cols = matrix_[0].size();

    std::vector<cost_t> min_values(cols, std::numeric_limits<cost_t>::max());

    // Code to populate min_values, assuming the logic is to find the minimum in each column
    for (std::size_t j = 0; j < cols; ++j) {
        for (std::size_t i = 0; i < rows; ++i) {
            min_values[j] = std::min(min_values[j], matrix_[i][j]);
        }
    }

    return min_values;

    
}

cost_t CostMatrix::reduce_cols() {
    if (matrix_.empty())
        return 0;

    cost_t total_reduction = 0;
    auto min_values = get_min_values_in_cols();

    std::size_t rows = matrix_.size();
    std::size_t cols = matrix_[0].size();

    for (std::size_t j = 0; j < cols; ++j) {
        cost_t min_val = min_values[j];

        if (min_val == std::numeric_limits<cost_t>::max() || min_val == 0)
            continue;

        total_reduction += min_val;

        for (std::size_t i = 0; i < rows; ++i) {
            if (matrix_[i][j] != std::numeric_limits<cost_t>::max()) {
                matrix_[i][j] -= min_val;
            }
        }
    }

    return total_reduction;
}

cost_t CostMatrix::get_vertex_cost(std::size_t row, std::size_t col) const {
    if (matrix_.empty())
        return 0;

    cost_t row_min = std::numeric_limits<cost_t>::max();
    cost_t col_min = std::numeric_limits<cost_t>::max();

    for (std::size_t j = 0; j < matrix_[row].size(); ++j) {
        if (j == col) continue;
        if (matrix_[row][j] < row_min)
            row_min = matrix_[row][j];
    }

    for (std::size_t i = 0; i < matrix_.size(); ++i) {
        if (i == row) continue;
        if (matrix_[i][col] < col_min)
            col_min = matrix_[i][col];
    }

    if (row_min == std::numeric_limits<cost_t>::max())
        row_min = 0;
    if (col_min == std::numeric_limits<cost_t>::max())
        col_min = 0;

    return row_min + col_min;
}

/* PART 2 */

/**
 * Choose next vertex to visit:
 * - Look for vertex_t (pair row and column) with value 0 in the current cost matrix.
 * - Get the vertex_t cost (calls get_vertex_cost()).
 * - Choose the vertex_t with maximum cost and returns it.
 * @param cm
 * @return The coordinates of the next vertex.
 */
NewVertex StageState::choose_new_vertex() {
    cost_t max_cost = -1;

    vertex_t next_vertex = {
        std::numeric_limits<std::size_t>::max(),
        std::numeric_limits<std::size_t>::max()
    };

    for (std::size_t i = 0; i < matrix_.size(); ++i) {
        for (std::size_t j = 0; j < matrix_[i].size(); ++j) {
            if (matrix_[i][j] == 0) {
                cost_t vertex_cost = matrix_.get_vertex_cost(i, j);
                if (vertex_cost > max_cost) {
                    max_cost = vertex_cost;
                    next_vertex = {i, j};
                }
            }
        }
    }

    return NewVertex{next_vertex, max_cost}; // Return the next vertex and its cost
}

/**
 * Update the cost matrix with the new vertex.
 * @param new_vertex
 */
void StageState::update_cost_matrix(vertex_t new_vertex) {

    matrix_[new_vertex.row][new_vertex.col] = INF;

    for (std::size_t i = 0; i < matrix_.size(); ++i) {
        matrix_[i][new_vertex.col] = INF; // Column
        matrix_[new_vertex.row][i] = INF; // Row
    }
}

/**
 * Reduce the cost matrix.
 * @return The sum of reduced values.
 */
cost_t StageState::reduce_cost_matrix() {

    cost_t row_reduction = matrix_.reduce_rows();
    cost_t col_reduction = matrix_.reduce_cols();

    return row_reduction + col_reduction;
}

/**
 * Given the optimal path, return the optimal cost.
 * @param optimal_path
 * @param m
 * @return Cost of the path.
 */
cost_t get_optimal_cost(const path_t& optimal_path, const cost_matrix_t& m) {
    cost_t cost = 0;

    for (std::size_t idx = 1; idx < optimal_path.size(); ++idx) {
        cost += m[optimal_path[idx - 1]][optimal_path[idx]];
    }

    cost += m[optimal_path[optimal_path.size() - 1]][optimal_path[0]];

    return cost;
}

/**
 * Create the right branch matrix with the chosen vertex forbidden and the new lower bound.
 * @param m
 * @param v
 * @param lb
 * @return New branch.
 */
StageState create_right_branch_matrix(cost_matrix_t m, vertex_t v, cost_t lb) {
    CostMatrix cm(m);
    cm[v.row][v.col] = INF;
    return StageState(cm, {}, lb);
}

/**
 * Retain only optimal ones (from all possible ones).
 * @param solutions
 * @return Vector of optimal solutions.
 */
tsp_solutions_t filter_solutions(tsp_solutions_t solutions) {
    cost_t optimal_cost = INF;
    for (const auto& s : solutions) {
        optimal_cost = (s.lower_bound < optimal_cost) ? s.lower_bound : optimal_cost;
    }

    tsp_solutions_t optimal_solutions;
    std::copy_if(solutions.begin(), solutions.end(),
                 std::back_inserter(optimal_solutions),
                 [&optimal_cost](const tsp_solution_t& s) { return s.lower_bound == optimal_cost; }
    );

    return optimal_solutions;
}

/**
 * Solve the TSP.
 * @param cm The cost matrix.
 * @return A list of optimal solutions.
 */
tsp_solutions_t solve_tsp(const cost_matrix_t& cm) {

    StageState left_branch(cm);

    // The branch & bound tree.
    std::stack<StageState> tree_lifo;

    // The number of levels determines the number of steps before obtaining
    // a 2x2 matrix.
    std::size_t n_levels = cm.size() - 2;

    tree_lifo.push(left_branch);   // Use the first cost matrix as the root.

    cost_t best_lb = INF;
    tsp_solutions_t solutions;

    while (!tree_lifo.empty()) {

        left_branch = tree_lifo.top();
        tree_lifo.pop();

        while (left_branch.get_level() != n_levels && left_branch.get_lower_bound() <= best_lb) {
            // Repeat until a 2x2 matrix is obtained or the lower bound is too high...

            if (left_branch.get_level() == 0) {
                left_branch.reset_lower_bound();
            }

            // 1. Reduce the matrix in rows and columns.
            cost_t new_cost = left_branch.reduce_cost_matrix();

            // 2. Update the lower bound and check the break condition.
            left_branch.update_lower_bound(new_cost);
            if (left_branch.get_lower_bound() > best_lb) {
                break;
            }

            // 3. Get new vertex and the cost of not choosing it.
            NewVertex new_vertex = left_branch.choose_new_vertex(); // @TODO (KROK 2)


            // 4. @TODO Update the path - use append_to_path method.

            left_branch.append_to_path(new_vertex.coordinates);  // @TODO (KROK 3)
            left_branch.update_cost_matrix(new_vertex.coordinates);


            // 6. Update the right branch and push it to the LIFO.
            cost_t new_lower_bound = left_branch.get_lower_bound() + new_vertex.cost;
            tree_lifo.push(create_right_branch_matrix(cm, new_vertex.coordinates,
                                                      new_lower_bound));
        }

        if (left_branch.get_lower_bound() <= best_lb) {
            // If the new solution is at least as good as the previous one,
            // save its lower bound and its path.
            best_lb = left_branch.get_lower_bound();
            path_t new_path = left_branch.get_path();
            solutions.push_back({get_optimal_cost(new_path, cm), new_path});
        }
    }

    return filter_solutions(solutions); // Filter solutions to find only optimal ones.
}
