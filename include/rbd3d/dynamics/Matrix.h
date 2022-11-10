#ifndef RBD3D_DYNAMICS_MATRIX_H_
#define RBD3D_DYNAMICS_MATRIX_H_

#include <cassert>
#include <vector>

namespace rbd3d
{
template <typename Ty>
class Matrix
{
public:
    Matrix() = default;
    Matrix(const Matrix &) = delete;
    Matrix &operator=(const Matrix &) = delete;

    size_t row() const { return m_row; }
    size_t col() const { return m_col; }

    void resize(size_t row, size_t col)
    {
        size_t sz = row * col;
        if (sz > m_buffer.size())
            m_buffer.resize(sz);
        m_row = row;
        m_col = col;
    }

    void resize(size_t n)
    {
        if (n > m_buffer.size())
            m_buffer.resize(n);
        m_row = n;
        m_col = 1;
    }

    Ty &operator()(size_t row, size_t col)
    {
        assert((row < m_row) && (col < m_col));
        return m_buffer[m_col * row + col];
    }
    const Ty operator()(size_t row, size_t col) const
    {
        assert((row < m_row) && (col < m_col));
        return m_buffer[m_col * row + col];
    }

    Ty &operator()(size_t i)
    {
        assert((m_col == 1) && (i < m_row));
        return m_buffer[i];
    }
    const Ty operator()(size_t i) const
    {
        assert((m_col == 1) && (i < m_row));
        return m_buffer[i];
    }

private:
    size_t m_row;
    size_t m_col;
    std::vector<Ty> m_buffer;
};
} // namespace rbd3d

#endif