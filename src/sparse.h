#ifndef SPARSE_H
#define SPARSE_H

/* sparse matrix data structure -----------------------------------------------
 *
 * The sparse matrix is stored in Compressed Sparse Column (CSC) format
 * The data is stored in column-major order
 * The row indices are sorted in ascending order
 * A reference to more information on CSC format:
 * https://en.wikipedia.org/wiki/Sparse_matrix#Compressed_sparse_column_(CSC_or_CCS)
 */
typedef struct {
  int n_rows;       /* number of rows */
  int n_cols;       /* number of columns */
  int nnz;          /* number of non-zero elements */
  int capacity;     /* current capacity of data and row_indices arrays */
  int* row_indices; /* array containing the row indices of non-zero elements */
  double* data;     /* array of non-zero elements */
  int* col_starts;  /* array containing the indices of the first non-zero element in each column */
} sparse_mat_t;

/* allocate sparse matrix -----------------------------------------------------*/
sparse_mat_t* sparse_mat_create(int n_rows, int n_cols);

/* allocate sparse matrix with initial capacity -------------------------------*/
sparse_mat_t* sparse_mat_create_with_capacity(int n_rows, int n_cols, int capacity);

/* free sparse matrix ---------------------------------------------------------*/
void sparse_mat_free(sparse_mat_t* A);

/* Set element in sparse matrix  -------------------
 *
 * Random access is supported
 * (i.e. elements can be set in any order)
 * if element already exists, it is overwritten
 * makes sure that the row indices are sorted in ascending order
 * make sure to call sparse_mat_compress() after setting all elements
 */
void sparse_mat_set_element(sparse_mat_t* A, int row, int col, double value);

/* get element from sparse matrix ---------------------------------------------*/
double sparse_mat_get_element(const sparse_mat_t* const A, int row, int col);

/* compress sparse matrix -----------------------------------------------------*/
void sparse_mat_compress(sparse_mat_t* A);

#endif
