import numpy as np
import os


def convert_agents_fname_to_priorities_fname(agents_fname: str) -> str:
    return agents_fname[:-7] + '.priorities'


def read_priorities_file(priorities_fname: str) -> np.array:
    with open(priorities_fname, 'r') as f:
        l = [[num for num in line.split('\n')][0] for line in f]
    adjacency_matrix = [[1 if elem == '@' else 0 for elem in row] for row in l]

    # for idx, line in enumerate(adjacency_matrix):
    #     print(f'{idx}: {line}')

    # print(np.array(adjacency_matrix))
    return np.array(adjacency_matrix)


def topological_sort(adj_mat: np.array):
    import copy
    adj_mat_ = copy.deepcopy(adj_mat)
    sum_per_col = np.sum(adj_mat_, 0)
    # print(f'sum_per_col = {sum_per_col}')
    k = len(adj_mat_)  # = len(adj_mat[0]) also, kxk matrix.
    completed = 0
    total_order = []  # ID_i < ID_j < ...
    while completed != k:
        if np.min(sum_per_col) != 0:  # cycle
            # input('cycle detected!!!')
            sum_per_col[np.argmin(sum_per_col)] = 0

        for i in range(k):
            if sum_per_col[i] == 0:  # no one better than i
                # print(f'adj_mat_[i={i}] before = \n {adj_mat_[i]}')
                total_order.append(i)
                adj_mat_[i] = 0  # zero row
                sum_per_col = np.sum(adj_mat_, 0)
                sum_per_col[total_order] = k  # finished
                completed = len(total_order)
                # print(f'adj_mat=\n')
                # [print(row) for row in adj_mat_]
                # print(f'sum_per_col = {sum_per_col}')
                # print(f'completed = {completed}')
                # print(f'total_order = {total_order}')
                # # print(f'adj_mat_[i={i}] = \n {adj_mat_[i]}')
                # input('?')

    return total_order


def create_adjacency_mat_from_top_sort(top_sort):
    k = len(top_sort)
    adj_mat = np.zeros((k, k))
    for i in range(k - 1):
        adj_mat[top_sort[i], top_sort[i + 1]] = 1
        for j in range(i, k - 1):
            adj_mat[top_sort[i], top_sort[j + 1]] = 1
    return adj_mat


def write_topological_sort_in_file(output_fname: str, top_sort_adj_matrix: np.array):
    with open(output_fname, 'w') as f:
        for i in range(len(top_sort_adj_matrix)):
            for j in range(len(top_sort_adj_matrix)):
                f.write('@' if top_sort_adj_matrix[i][j] == 1 else '.')
            f.write('\n')


def override_priorities_file_with_topological_sort(agents_fname: str):
    priorities_fname = convert_agents_fname_to_priorities_fname(agents_fname)
    folder_fname = os.path.dirname(os.path.realpath(__file__))
    os.path.join(os.path.dirname(folder_fname), agents_fname)
    adjacency_matrix = read_priorities_file(priorities_fname=priorities_fname)
    # print(f'adjacency_matrix = \n{adjacency_matrix}')
    # print(f'adjacency_matrix = \n{np.sum(adjacency_matrix)}')
    # input('?')
    topological_sorted = topological_sort(adj_mat=adjacency_matrix)
    topological_sorted_adjacency_matrix = create_adjacency_mat_from_top_sort(topological_sorted)
    # input('write file???')
    write_topological_sort_in_file(output_fname=priorities_fname,
                                   top_sort_adj_matrix=topological_sorted_adjacency_matrix)


if __name__ == '__main__':
    agents_fname = '../files/kiva_33x46/170agents/lifelong/kiva_33x46map-170agents-_w.agents'
    override_priorities_file_with_topological_sort(agents_fname=agents_fname)
    # priorities_fname = convert_agents_fname_to_priorities_fname(agents_fname)
    # folder_fname = os.path.dirname(os.path.realpath(__file__))
    # path.join(path.dirname(folder_fname), agents_fname)
    # adjacency_matrix = read_priorities_file(priorities_fname=priorities_fname)
    #
    # topological_sorted = topological_sort(adj_mat=adjacency_matrix)
    # topological_sorted_adjacency_matrix = create_adjacency_mat_from_top_sort(topological_sorted)
    # write_topological_sort_in_file(output_fname='abc.priorities', top_sort_adj_matrix=topological_sorted_adjacency_matrix)
