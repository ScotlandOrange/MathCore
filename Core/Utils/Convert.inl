#pragma once

#include <cassert>
#include <sstream>
#include <type_traits>

namespace ZF
{

template <typename Out,
          std::size_t N,
          typename Derived,
          enable_if_eigen_vector_t<Derived>,
          enable_if_arithmetic_t<Out>>
inline detail::ConvertStdArray_t<Out, N, Derived> toStdArray(const Eigen::MatrixBase<Derived>& vec)
{
    constexpr std::size_t Dimension = static_cast<std::size_t>(Derived::SizeAtCompileTime);
    static_assert(N == Dimension,
                  "toStdArray<Out, N>(vec) requires N to match vector dimension");

    detail::ConvertStdArray_t<Out, N, Derived> result{};
    for (std::size_t i = 0; i < Dimension; ++i)
    {
        result[i] = static_cast<Out>(vec.derived().coeff(static_cast<Eigen::Index>(i)));
    }
    return result;
}

template <std::size_t N,
          typename Derived,
          enable_if_eigen_vector_t<Derived>>
inline detail::ConvertStdArray_t<void, N, Derived> toStdArray(const Eigen::MatrixBase<Derived>& vec)
{
    return toStdArray<eigen_scalar_t<Derived>, N>(vec);
}

template <typename Out,
          std::size_t Rows,
          std::size_t Cols,
          typename Derived,
          enable_if_eigen_matrix_t<Derived>,
          enable_if_arithmetic_t<Out>>
inline detail::ConvertStdMatrixArray_t<Out, Rows, Cols, Derived> toStdArray(const Eigen::MatrixBase<Derived>& m)
{
    constexpr std::size_t MatrixRows = static_cast<std::size_t>(Derived::RowsAtCompileTime);
    constexpr std::size_t MatrixCols = static_cast<std::size_t>(Derived::ColsAtCompileTime);
    static_assert(Rows == MatrixRows,
                  "toStdArray<Out, Rows, Cols>(m) requires Rows to match matrix row count");
    static_assert(Cols == MatrixCols,
                  "toStdArray<Out, Rows, Cols>(m) requires Cols to match matrix column count");

    detail::ConvertStdMatrixArray_t<Out, Rows, Cols, Derived> result{};
    for (std::size_t r = 0; r < MatrixRows; ++r)
    {
        for (std::size_t c = 0; c < MatrixCols; ++c)
        {
            result[r][c] = static_cast<Out>(m.derived().coeff(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c)));
        }
    }
    return result;
}

template <std::size_t Rows,
          std::size_t Cols,
          typename Derived,
          enable_if_eigen_matrix_t<Derived>>
inline detail::ConvertStdMatrixArray_t<void, Rows, Cols, Derived> toStdArray(const Eigen::MatrixBase<Derived>& m)
{
    return toStdArray<eigen_scalar_t<Derived>, Rows, Cols>(m);
}

template <typename Out,
          std::size_t N,
          typename Derived,
          enable_if_eigen_quaternion_t<Derived>,
          enable_if_arithmetic_t<Out>>
inline detail::ConvertStdArray_t<Out, N, Derived> toStdArray(const Eigen::QuaternionBase<Derived>& q)
{
    static_assert(N == 4,
                  "toStdArray<Out, N>(q) requires N to be 4");

    return detail::ConvertStdArray_t<Out, N, Derived>{
        static_cast<Out>(q.x()),
        static_cast<Out>(q.y()),
        static_cast<Out>(q.z()),
        static_cast<Out>(q.w()),
    };
}

template <std::size_t N,
          typename Derived,
          enable_if_eigen_quaternion_t<Derived>>
inline detail::ConvertStdArray_t<void, N, Derived> toStdArray(const Eigen::QuaternionBase<Derived>& q)
{
    return toStdArray<eigen_scalar_t<Derived>, N>(q);
}

template <typename Out, typename Derived, enable_if_eigen_vector_t<Derived>, enable_if_arithmetic_t<detail::ConvertOutputScalar_t<Out, Derived>>>
inline detail::ConvertStdVector_t<Out, Derived> toStdVector(const Eigen::MatrixBase<Derived>& vec)
{
    constexpr std::size_t N = static_cast<std::size_t>(Derived::SizeAtCompileTime);

    detail::ConvertStdVector_t<Out, Derived> result(N);
    for (std::size_t i = 0; i < N; ++i)
    {
        result[i] = static_cast<detail::ConvertOutputScalar_t<Out, Derived>>(vec.derived().coeff(static_cast<Eigen::Index>(i)));
    }
    return result;
}

template <typename Out, typename Derived, enable_if_eigen_matrix_t<Derived>, enable_if_arithmetic_t<detail::ConvertOutputScalar_t<Out, Derived>>>
inline detail::ConvertStdVector_t<Out, Derived> MatrixToStdVector1D(const Eigen::MatrixBase<Derived>& m,
                                                                    MatrixToStdVectorOrder order)
{
    constexpr std::size_t Rows = static_cast<std::size_t>(Derived::RowsAtCompileTime);
    constexpr std::size_t Cols = static_cast<std::size_t>(Derived::ColsAtCompileTime);
    using OutputScalar = detail::ConvertOutputScalar_t<Out, Derived>;

    detail::ConvertStdVector_t<Out, Derived> out;
    out.reserve(Rows * Cols);

    if (order == MatrixToStdVectorOrder::RowMajor)
    {
        for (std::size_t r = 0; r < Rows; ++r)
        {
            for (std::size_t c = 0; c < Cols; ++c)
            {
                out.push_back(static_cast<OutputScalar>(
                    m.derived().coeff(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c))));
            }
        }
        return out;
    }

    for (std::size_t c = 0; c < Cols; ++c)
    {
        for (std::size_t r = 0; r < Rows; ++r)
        {
            out.push_back(static_cast<OutputScalar>(
                m.derived().coeff(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c))));
        }
    }
    return out;
}

template <typename Out, typename Derived, enable_if_eigen_matrix_t<Derived>, enable_if_arithmetic_t<detail::ConvertOutputScalar_t<Out, Derived>>>
inline detail::ConvertStdMatrixVector_t<Out, Derived> MatrixToStdVector2D(const Eigen::MatrixBase<Derived>& m,
                                                                          MatrixToStdVectorOrder order)
{
    constexpr std::size_t Rows = static_cast<std::size_t>(Derived::RowsAtCompileTime);
    constexpr std::size_t Cols = static_cast<std::size_t>(Derived::ColsAtCompileTime);
    using OutputScalar = detail::ConvertOutputScalar_t<Out, Derived>;

    if (order == MatrixToStdVectorOrder::RowMajor)
    {
        detail::ConvertStdMatrixVector_t<Out, Derived> out(Rows, detail::ConvertStdVector_t<Out, Derived>(Cols));
        for (std::size_t r = 0; r < Rows; ++r)
        {
            for (std::size_t c = 0; c < Cols; ++c)
            {
                out[r][c] = static_cast<OutputScalar>(
                    m.derived().coeff(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c)));
            }
        }
        return out;
    }

    detail::ConvertStdMatrixVector_t<Out, Derived> out(Cols, detail::ConvertStdVector_t<Out, Derived>(Rows));
    for (std::size_t c = 0; c < Cols; ++c)
    {
        for (std::size_t r = 0; r < Rows; ++r)
        {
            out[c][r] = static_cast<OutputScalar>(
                m.derived().coeff(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c)));
        }
    }
    return out;
}

template <typename Out, typename Derived, enable_if_eigen_quaternion_t<Derived>, enable_if_arithmetic_t<detail::ConvertOutputScalar_t<Out, Derived>>>
inline detail::ConvertStdVector_t<Out, Derived> toStdVector(const Eigen::QuaternionBase<Derived>& q)
{
    return detail::ConvertStdVector_t<Out, Derived>{
        static_cast<detail::ConvertOutputScalar_t<Out, Derived>>(q.x()),
        static_cast<detail::ConvertOutputScalar_t<Out, Derived>>(q.y()),
        static_cast<detail::ConvertOutputScalar_t<Out, Derived>>(q.z()),
        static_cast<detail::ConvertOutputScalar_t<Out, Derived>>(q.w()),
    };
}

template <typename Derived, typename U, std::size_t N, enable_if_eigen_vector_t<Derived>, enable_if_arithmetic_t<U>>
inline void toCArray(const Eigen::MatrixBase<Derived>& vec,
                     U (&out)[N])
{
    constexpr std::size_t Dim = static_cast<std::size_t>(Derived::SizeAtCompileTime);
    static_assert(N == Dim,
                  "toCArray(vec, out) requires output array size to match vector dimension");

    for (std::size_t i = 0; i < Dim; ++i)
    {
        out[i] = static_cast<U>(vec.derived().coeff(static_cast<Eigen::Index>(i)));
    }
}

template <typename Derived, typename U, std::size_t Rows, std::size_t Cols, enable_if_eigen_matrix_t<Derived>, enable_if_arithmetic_t<U>>
inline void toCArray(const Eigen::MatrixBase<Derived>& m,
                     U (&out)[Rows][Cols])
{
    constexpr std::size_t MatrixRows = static_cast<std::size_t>(Derived::RowsAtCompileTime);
    constexpr std::size_t MatrixCols = static_cast<std::size_t>(Derived::ColsAtCompileTime);
    static_assert(Rows == MatrixRows,
                  "toCArray(m, out) requires output row count to match matrix row count");
    static_assert(Cols == MatrixCols,
                  "toCArray(m, out) requires output column count to match matrix column count");

    for (std::size_t r = 0; r < MatrixRows; ++r)
    {
        for (std::size_t c = 0; c < MatrixCols; ++c)
        {
            out[r][c] = static_cast<U>(m.derived().coeff(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c)));
        }
    }
}

template <typename Derived, typename U, std::size_t N, enable_if_eigen_quaternion_t<Derived>, enable_if_arithmetic_t<U>>
inline void toCArray(const Eigen::QuaternionBase<Derived>& q,
                     U (&out)[N])
{
    static_assert(N == 4,
                  "toCArray(q, out) requires output array size to be 4");

    out[0] = static_cast<U>(q.x());
    out[1] = static_cast<U>(q.y());
    out[2] = static_cast<U>(q.z());
    out[3] = static_cast<U>(q.w());
}

template <typename Target, typename U, std::size_t N, enable_if_eigen_vector_t<Target>, enable_if_arithmetic_t<U>>
inline Target toEigenVector(const std::array<U, N>& arr)
{
    static_assert(Target::SizeAtCompileTime == static_cast<Eigen::Index>(N),
                  "toEigenVector<Target>(arr) requires input size to match target vector dimension");
    static_assert(std::is_arithmetic<eigen_scalar_t<Target>>{},
                  "toEigenVector<Target>(arr) requires arithmetic target scalar type");

    Target result;
    for (std::size_t i = 0; i < N; ++i)
    {
        result.coeffRef(static_cast<Eigen::Index>(i)) = static_cast<eigen_scalar_t<Target>>(arr[i]);
    }
    return result;
}

template <typename Target, typename U, std::size_t N, enable_if_eigen_vector_t<Target>, enable_if_arithmetic_t<U>>
inline Target toEigenVector(const U (&arr)[N])
{
    static_assert(Target::SizeAtCompileTime == static_cast<Eigen::Index>(N),
                  "toEigenVector<Target>(arr) requires input size to match target vector dimension");
    static_assert(std::is_arithmetic<eigen_scalar_t<Target>>{},
                  "toEigenVector<Target>(arr) requires arithmetic target scalar type");

    Target result;
    for (std::size_t i = 0; i < N; ++i)
    {
        result.coeffRef(static_cast<Eigen::Index>(i)) = static_cast<eigen_scalar_t<Target>>(arr[i]);
    }
    return result;
}

template <typename Target,
          typename U,
          enable_if_eigen_vector_t<Target>,
          std::enable_if_t<!std::is_void<std::remove_cv_t<U>>{} && std::is_arithmetic<std::remove_cv_t<U>>{}, int>>
inline Target toEigenVector(const U* const& arr)
{
    constexpr std::size_t N = static_cast<std::size_t>(Target::SizeAtCompileTime);
    static_assert(std::is_arithmetic<eigen_scalar_t<Target>>{},
                  "toEigenVector<Target>(arr) requires arithmetic target scalar type");
    static_assert(!std::is_void<std::remove_cv_t<U>>{},
                  "toEigenVector<Target>(arr) requires a non-void input scalar type");

    assert(arr != nullptr && "toEigenVector<Target>(arr) requires a non-null input pointer");

    Target result;
    for (std::size_t i = 0; i < N; ++i)
    {
        result.coeffRef(static_cast<Eigen::Index>(i)) = static_cast<eigen_scalar_t<Target>>(arr[i]);
    }
    return result;
}

template <typename Target, typename U, enable_if_eigen_vector_t<Target>, enable_if_arithmetic_t<U>>
inline Target toEigenVector(const std::vector<U>& arr)
{
    constexpr std::size_t N = static_cast<std::size_t>(Target::SizeAtCompileTime);
    static_assert(std::is_arithmetic<eigen_scalar_t<Target>>{},
                  "toEigenVector<Target>(arr) requires arithmetic target scalar type");

    assert(arr.size() == N && "toEigenVector<Target>(arr) requires input size to match target vector dimension");

    return toEigenVector<Target>(arr.data());
}

template <typename Target, enable_if_eigen_vector_t<Target>>
inline Target toEigenVector(const std::vector<std::string>& arr)
{
    constexpr std::size_t N = static_cast<std::size_t>(Target::SizeAtCompileTime);
    static_assert(std::is_arithmetic<eigen_scalar_t<Target>>{},
                  "toEigenVector<Target>(arr) requires arithmetic target scalar type");

    assert(arr.size() == N && "toEigenVector<Target>(arr) requires input size to match target vector dimension");

    Target result;
    for (std::size_t i = 0; i < N; ++i)
    {
        result.coeffRef(static_cast<Eigen::Index>(i)) = static_cast<eigen_scalar_t<Target>>(std::stod(arr[i]));
    }
    return result;
}

template <typename Target, enable_if_eigen_vector_t<Target>>
inline Target toEigenVector(const QStringList& arr)
{
    constexpr std::size_t N = static_cast<std::size_t>(Target::SizeAtCompileTime);
    static_assert(std::is_arithmetic<eigen_scalar_t<Target>>{},
                  "toEigenVector<Target>(arr) requires arithmetic target scalar type");

    assert(static_cast<std::size_t>(arr.size()) == N
           && "toEigenVector<Target>(arr) requires input size to match target vector dimension");

    Target result;
    for (std::size_t i = 0; i < N; ++i)
    {
        result.coeffRef(static_cast<Eigen::Index>(i)) = static_cast<eigen_scalar_t<Target>>(arr.at(static_cast<int>(i)).toDouble());
    }
    return result;
}

template <typename Derived, enable_if_eigen_vector_t<Derived>>
inline QJsonArray toQJsonArray(const Eigen::MatrixBase<Derived>& vec)
{
    ZF_ASSERT(vec.rows() == 1 || vec.cols() == 1);

    QJsonArray arr;
    for (Eigen::Index index = 0; index < vec.size(); ++index)
    {
        arr.append(static_cast<double>(vec.derived().coeff(index)));
    }

    return arr;
}

template <typename Derived, enable_if_eigen_matrix_t<Derived>>
inline QJsonArray toQJsonArray(const Eigen::MatrixBase<Derived>& m)
{
    QJsonArray arr;
    for (Eigen::Index row = 0; row < m.rows(); ++row)
    {
        QJsonArray rowArray;
        for (Eigen::Index col = 0; col < m.cols(); ++col)
        {
            rowArray.append(static_cast<double>(m.derived().coeff(row, col)));
        }

        arr.append(rowArray);
    }
    return arr;
}

template <typename Derived, enable_if_eigen_quaternion_t<Derived>>
inline QJsonArray toQJsonArray(const Eigen::QuaternionBase<Derived>& q)
{
    QJsonArray arr;
    arr.append(static_cast<double>(q.x()));
    arr.append(static_cast<double>(q.y()));
    arr.append(static_cast<double>(q.z()));
    arr.append(static_cast<double>(q.w()));
    return arr;
}

template <typename Derived, enable_if_eigen_vector_t<Derived>>
inline QString toQString(const Eigen::MatrixBase<Derived>& vec, const char* format)
{
    ZF_ASSERT(vec.rows() == 1 || vec.cols() == 1);
    ZF_ASSERT(format != nullptr);

    QString result = QString::fromLatin1(format);
    for (Eigen::Index index = 0; index < vec.size(); ++index)
    {
        result = result.arg(static_cast<double>(vec.derived().coeff(index)));
    }
    return result;
}

template <typename Derived, enable_if_eigen_matrix_any_t<Derived>>
inline QString toQString(const Eigen::MatrixBase<Derived>& m,
                         const QString& elementSeparator,
                         const QString& rowSeparator,
                         const QString& prefix,
                         const QString& suffix)
{
    QString result = prefix;
    for (Eigen::Index r = 0; r < m.rows(); ++r)
    {
        if (r > 0) result.append(rowSeparator);
        for (Eigen::Index c = 0; c < m.cols(); ++c)
        {
            if (c > 0) result.append(elementSeparator);
            result.append(QString::number(static_cast<double>(m.derived().coeff(r, c))));
        }
    }
    result.append(suffix);
    return result;
}

template <typename Derived, enable_if_eigen_quaternion_t<Derived>>
inline QString toQString(const Eigen::QuaternionBase<Derived>& q, const char* format)
{
    ZF_ASSERT(format != nullptr);
    return QString::fromLatin1(format)
           .arg(static_cast<double>(q.x()))
           .arg(static_cast<double>(q.y()))
           .arg(static_cast<double>(q.z()))
           .arg(static_cast<double>(q.w()));
}

template <typename Derived, enable_if_eigen_vector_t<Derived>>
inline std::string toStdString(const Eigen::MatrixBase<Derived>& vec,
                               const std::string& elementSeparator,
                               const std::string& prefix,
                               const std::string& suffix)
{
    ZF_ASSERT(vec.rows() == 1 || vec.cols() == 1);
    std::ostringstream os;
    os << prefix;
    for (Eigen::Index index = 0; index < vec.size(); ++index)
    {
        if (index > 0) os << elementSeparator;
        os << static_cast<double>(vec.derived().coeff(index));
    }
    os << suffix;
    return os.str();
}

template <typename Derived, enable_if_eigen_quaternion_t<Derived>>
inline std::string toStdString(const Eigen::QuaternionBase<Derived>& q,
                               const std::string& elementSeparator,
                               const std::string& prefix,
                               const std::string& suffix)
{
    std::ostringstream os;
    os << prefix
       << static_cast<double>(q.x()) << elementSeparator
       << static_cast<double>(q.y()) << elementSeparator
       << static_cast<double>(q.z()) << elementSeparator
       << static_cast<double>(q.w())
       << suffix;
    return os.str();
}

template <typename Derived, enable_if_eigen_matrix_any_t<Derived>>
inline std::string toStdString(const Eigen::MatrixBase<Derived>& m,
                               const std::string& elementSeparator,
                               const std::string& rowSeparator,
                               const std::string& prefix,
                               const std::string& suffix)
{
    std::ostringstream os;
    os << prefix;
    for (Eigen::Index r = 0; r < m.rows(); ++r)
    {
        if (r > 0) os << rowSeparator;
        for (Eigen::Index c = 0; c < m.cols(); ++c)
        {
            if (c > 0) os << elementSeparator;
            os << static_cast<double>(m.derived().coeff(r, c));
        }
    }
    os << suffix;
    return os.str();
}

} // namespace ZF
