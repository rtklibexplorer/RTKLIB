#include "sparse.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define sparse_mat_assert_index(func, A, row, col)                         \
  assert((row >= 0 && row < A->n_rows) && (col >= 0 && col < A->n_cols) && \
         func ": index out of range")

/* allocate sparse matrix -----------------------------------------------------*/
extern sparse_mat_t* sparse_mat_create(int n_rows, int n_cols)
{
  sparse_mat_t* A = (sparse_mat_t*)malloc(sizeof(sparse_mat_t));
  A->n_rows = n_rows;
  A->n_cols = n_cols;
  A->nnz = 0;
  A->capacity = 0;
  A->col_starts = (int*)calloc(n_cols + 1, sizeof(int));
  A->row_indices = NULL;
  A->data = NULL;
  return A;
}

/* allocate sparse matrix with initial capacity -------------------------------*/
extern sparse_mat_t* sparse_mat_create_with_capacity(int n_rows, int n_cols, int capacity)
{
  sparse_mat_t* A = sparse_mat_create(n_rows, n_cols);
  A->capacity = capacity;
  A->row_indices = (int*)calloc(capacity, sizeof(int));
  A->data = (double*)calloc(capacity, sizeof(double));
  return A;
}

/* free sparse matrix ---------------------------------------------------------*/
extern void sparse_mat_free(sparse_mat_t* A)
{
  if (A) {
    free(A->col_starts);
    if (A->row_indices) free(A->row_indices);
    if (A->data) free(A->data);
    free(A);
  }
}

/* get index of element in sparse matrix --------------------------------------
 * returns index in data array if element exists
 * returns index where element should be inserted if element does not exist
 */
static int sparse_mat_get_element_index(const sparse_mat_t* const A, int row, int col)
{
  int k;
  sparse_mat_assert_index("get_sparse_mat_element_index", A, row, col);

  k = A->col_starts[col];
  while (k < A->col_starts[col + 1] && A->row_indices[k] < row) k++;
  return k;
}

/* get element from sparse matrix ---------------------------------------------*/
extern double sparse_mat_get_element(const sparse_mat_t* const A, int row, int col)
{
  int k, exists;
  sparse_mat_assert_index("get_sparse_mat_element", A, row, col);

  k = sparse_mat_get_element_index(A, row, col);
  exists = k < A->col_starts[col + 1] && A->row_indices[k] == row;
  return exists ? A->data[k] : 0.0;
}

/* Remove sparse matrix element -----------------------------------------------
 *
 * Assumes that the element exists
 */
static void sparse_mat_remove_element(sparse_mat_t* A, int k, int col)
{
  int i;
  int start = col + 1;
  int n_elements = A->n_cols - col;
  /* shift elements to the left */
  const size_t n_elements_to_move = A->col_starts[A->n_cols] - k - 1;
  memmove(A->row_indices + k, A->row_indices + k + 1, n_elements_to_move * sizeof(int));
  memmove(A->data + k, A->data + k + 1, n_elements_to_move * sizeof(double));

  /* decrement start indices of subsequent columns
   * precomputing the start and n_elements to set, enables auto-vectorization */
  for (i = 0; i < n_elements; i++) A->col_starts[start + i]--;
  A->nnz--;
}

/* Insert element into sparse matrix -----------------------------------------------
 *
 * Assumes that the element does not exist
 * Assumes that there is enough capacity to insert the element
 */
static void sparse_mat_insert_element(sparse_mat_t* A, int k, int row, int col, double value)
{
  int i;
  int start = col + 1;
  int n_elements = A->n_cols - col;
  /* shift elements to the right */
  const size_t n_elements_to_move = A->col_starts[A->n_cols] - k;
  memmove(A->row_indices + k + 1, A->row_indices + k, n_elements_to_move * sizeof(int));
  memmove(A->data + k + 1, A->data + k, n_elements_to_move * sizeof(double));

  /* insert element */
  A->row_indices[k] = row;
  A->data[k] = value;

  /* increment start indices of subsequent columns
   * precomputing the start and n_elements to set, enables auto-vectorization */
  for (i = 0; i < n_elements; i++) A->col_starts[start + i]++;
  A->nnz++;
}

/* Set element in sparse matrix  -------------------
 *
 * Random access is supported
 * (i.e. elements can be set in any order)
 * if element already exists, it is overwritten
 * if value is zero, the element is removed, if it exists and not inserted if it does not exist
 * makes sure that the row indices are sorted in ascending order
 * make sure to call compress_sparse_mat() after setting all elements
 */
extern void sparse_mat_set_element(sparse_mat_t* A, int row, int col, double value)
{
  int k, exists;
  sparse_mat_assert_index("set_sparse_mat_element", A, row, col);

  /* find the position of the element in the column */
  k = sparse_mat_get_element_index(A, row, col);

  /* assign value if element already exists */
  exists = k < A->col_starts[col + 1] && A->row_indices[k] == row;
  if (exists) {
    if (value != 0.0) {
      A->data[k] = value;
      return;
    }
    /* remove element if value is zero */
    sparse_mat_remove_element(A, k, col);
    return;
  }

  if (value == 0.0) return;

  /* grow arrays if necessary */
  if (A->nnz == A->capacity) {
    A->capacity = A->capacity == 0 ? 1 : 2 * A->capacity;
    A->row_indices = (int*)realloc(A->row_indices, A->capacity * sizeof(int));
    A->data = (double*)realloc(A->data, A->capacity * sizeof(double));
  }

  /* insert element if it does not exist */
  sparse_mat_insert_element(A, k, row, col, value);
}

/* compress sparse matrix -----------------------------------------------------*/
extern void sparse_mat_compress(sparse_mat_t* A)
{
  /* shrink arrays to fit */
  A->capacity = A->nnz;
  A->row_indices = (int*)realloc(A->row_indices, A->capacity * sizeof(int));
  A->data = (double*)realloc(A->data, A->capacity * sizeof(double));
}
