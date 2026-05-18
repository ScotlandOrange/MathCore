#include <Core/Utils/Convert.h>

namespace ZF
{

QStringList toStringList(const std::vector<std::string>& vec)
{
    QStringList result;
    result.reserve(static_cast<qsizetype>(vec.size()));
    for (const auto& item : vec)
    {
        result.append(QString::fromStdString(item));
    }
    return result;
}

std::vector<std::string> toVector(const QStringList& list)
{
    std::vector<std::string> result;
    result.reserve(static_cast<std::size_t>(list.size()));
    for (const auto& item : list)
    {
        result.emplace_back(item.toStdString());
    }
    return result;
}

} // namespace ZF