#pragma once

#include <tailor/tailor.h>
#include <tailor/tailor_concept.h>
#include <tailor/tailor_point.h>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

TAILOR_NAMESPACE_BEGIN

/**
 * @brief 多边形序列化/反序列化工具 + Drafting 序列化
 *
 * 支持文本格式（与测试数据兼容）和二进制格式。
 * 通过 EdgeDataTraits 可从任意边类型提取几何数据，也支持用户数据序列化。
 *
 * 文本格式:
 *   # 注释行
 *   BEGIN_POLYGON N
 *   startX startY endX endY bulge
 *   ...
 *   END_POLYGON
 *
 * 二进制格式:
 *   Header: magic(4B) + version(1B) + flags(1B) + reserved(2B)
 *   Body:   polygonCount(4B) + [edgeCount(4B) + edgeData...]...
 *   每条边: sx sy ex ey (固定) + bulge(可选) + userDataSize(4B,可选) + userData(可选)
 *
 * 使用示例:
 *   @code
 *   using MyEdge = tailor::ArcSegment<MyPoint, double, MyUserData>;
 *   tailor::polygon_io::SerializedPolygon<double> sp;
 *   // ... 读写 ...
 *   std::vector<MyEdge> edges = tailor::polygon_io::ToEdges<MyEdge>(sp);
 *
 *   // Drafting 序列化:
 *   auto drafting = tailor.Execute();
 *   polygon_io::WriteDraftingFile<Arc>("drafting.bin", drafting.edgeEvent, drafting.vertexEvents);
 *   // ... 反序列化 ...
 *   std::vector<EdgeEvent<Arc>> edgeEvents;
 *   std::vector<TopoVertex> vertexEvents;
 *   polygon_io::ReadDraftingFile<Arc>("drafting.bin", edgeEvents, vertexEvents);
 *   @endcode
 */
	namespace polygon_io {
// ============================================================================
// 序列化用纯数据类型
// ============================================================================

/** 序列化边 —— 不依赖具体边类型，仅保留几何与bulge */
template <typename CoordType = double>
struct SerializedEdge {
	CoordType sx = 0, sy = 0;   // 起点
	CoordType ex = 0, ey = 0;   // 终点
	double    bulge = 0.0;       // bulge（0 = 线段, ≠0 = 弧线）
};

template <typename CoordType = double>
using SerializedPolygon = std::vector<SerializedEdge<CoordType>>;

// ============================================================================
// EdgeDataTraits —— 从任意边类型提取/设置几何数据
// ============================================================================

/**
 * @brief 默认边数据萃取器
 *
 * 假设边类型提供 Point0(), Point1(), 以及可选的 Bulge(), Data()
 * 用户可通过特化来支持自定义边类型。
 *
 * @tparam Edge    边类型
 * @tparam Enable  SFINAE 占位
 */
template <typename Edge, typename Enable = void>
struct EdgeDataTraits {
	using EdgeType = Edge;
	using PointType = typename Edge::PointType;
	using CoordType = typename PointTraits<PointType>::CoordinateType;
	using UserDataType = typename Edge::UserDataType;

	// ---- 类型检测 ----
	static constexpr bool HasBulge() {
		return requires { std::declval<EdgeType&>().Bulge(); };
	}
	static constexpr bool HasUserData() {
		return !std::is_same_v<UserDataType, void>;
	}

	// ---- 提取 ----
	static CoordType GetStartX(const EdgeType& e) { return e.Point0().x; }
	static CoordType GetStartY(const EdgeType& e) { return e.Point0().y; }
	static CoordType GetEndX(const EdgeType& e) { return e.Point1().x; }
	static CoordType GetEndY(const EdgeType& e) { return e.Point1().y; }

	static double GetBulge(const EdgeType& e) {
		if constexpr (HasBulge())
			return static_cast<double>(e.Bulge());
		else
			return 0.0;
	}

	// ---- 用户数据序列化接口（按需特化） ----
	/// 将用户数据序列化为字节流（默认: 按 POD 直接拷贝）
	static std::vector<uint8_t> SerializeUserData(const EdgeType& e) {
		if constexpr (HasUserData()) {
			if constexpr (std::is_trivially_copyable_v<UserDataType>) {
				std::vector<uint8_t> buf(sizeof(UserDataType));
				std::memcpy(buf.data(), &e.Data(), sizeof(UserDataType));
				return buf;
			} else {
				static_assert(sizeof(UserDataType) == 0,
					"Non-trivially-copyable UserData requires a custom EdgeDataTraits specialization.");
				return {};
			}
		}
		return {};
	}

	/// 从字节流反序列化用户数据（默认: 按 POD 直接拷贝）
	static void DeserializeUserData(EdgeType& e, const std::vector<uint8_t>& buf) {
		if constexpr (HasUserData()) {
			if constexpr (std::is_trivially_copyable_v<UserDataType>) {
				if (buf.size() >= sizeof(UserDataType))
					std::memcpy(&e.Data(), buf.data(), sizeof(UserDataType));
			} else {
				static_assert(sizeof(UserDataType) == 0,
					"Non-trivially-copyable UserData requires a custom EdgeDataTraits specialization.");
			}
		}
	}

	// ---- 构造 ----
	/// 构造辅助：带 bulge 参数（用于 ArcSegment 等）
	template <typename CoordT>
	static EdgeType ConstructWithBulge(CoordT sx, CoordT sy, CoordT ex, CoordT ey,
		double bulge, const std::vector<uint8_t>& userData) {
		using BulgeType = std::decay_t<decltype(std::declval<EdgeType&>().Bulge())>;
		PointType p0(sx, sy);
		PointType p1(ex, ey);
		if constexpr (HasUserData()) {
			UserDataType ud{};
			if (userData.size() >= sizeof(UserDataType))
				std::memcpy(&ud, userData.data(), sizeof(UserDataType));
			return EdgeType(p0, p1, static_cast<BulgeType>(bulge), ud);
		} else {
			(void)userData;
			return EdgeType(p0, p1, static_cast<BulgeType>(bulge));
		}
	}

	/// 构造辅助：无 bulge 参数（用于 LineSegment 等）
	template <typename CoordT>
	static EdgeType ConstructWithoutBulge(CoordT sx, CoordT sy, CoordT ex, CoordT ey,
		double, const std::vector<uint8_t>& userData) {
		PointType p0(sx, sy);
		PointType p1(ex, ey);
		if constexpr (HasUserData()) {
			UserDataType ud{};
			if (userData.size() >= sizeof(UserDataType))
				std::memcpy(&ud, userData.data(), sizeof(UserDataType));
			return EdgeType(p0, p1, ud);
		} else {
			return EdgeType(p0, p1);
		}
	}

	/// 统一构造入口（根据边是否有 bulge 自动分派）
	template <typename CoordT>
	static EdgeType Construct(
		CoordT sx, CoordT sy, CoordT ex, CoordT ey, double bulge,
		const std::vector<uint8_t>& userData) {
		if constexpr (HasBulge())
			return ConstructWithBulge(sx, sy, ex, ey, bulge, userData);
		else
			return ConstructWithoutBulge(sx, sy, ex, ey, bulge, userData);
	}
};

// ============================================================================
// 文本格式 IO
// ============================================================================

/**
 * @brief 文本格式多边形读写器
 * @tparam CoordType  坐标数值类型
 */
template <typename CoordType = double>
class TextPolygonIO {
public:
	// ---- 辅助：字符串处理 ----
	static std::string Trim(const std::string& s) {
		size_t b = 0, e = s.size();
		while (b < e && (s[b] == ' ' || s[b] == '\t' || s[b] == '\r')) ++b;
		while (b < e && (s[e - 1] == ' ' || s[e - 1] == '\t' || s[e - 1] == '\r')) --e;
		return s.substr(b, e - b);
	}

	static std::vector<std::string> SplitTokens(const std::string& line) {
		std::vector<std::string> tokens;
		std::istringstream iss(line);
		std::string token;
		while (iss >> token) tokens.push_back(token);
		return tokens;
	}

	static bool ParseEdge(const std::vector<std::string>& tokens, SerializedEdge<CoordType>& out) {
		if (tokens.size() < 5) return false;
		out.sx = static_cast<CoordType>(std::stod(tokens[0]));
		out.sy = static_cast<CoordType>(std::stod(tokens[1]));
		out.ex = static_cast<CoordType>(std::stod(tokens[2]));
		out.ey = static_cast<CoordType>(std::stod(tokens[3]));
		out.bulge = std::stod(tokens[4]);
		return true;
	}

	// ---- 读取 ----
	/**
	 * @brief 从输入流读取多边形数据
	 * @param is  输入流
	 * @param out 输出多边形列表
	 * @return 是否成功读入至少一个多边形
	 */
	static bool Read(std::istream& is, std::vector<SerializedPolygon<CoordType>>& out) {
		out.clear();
		std::vector<SerializedEdge<CoordType>> currentEdges;
		bool inPolygon = false;

		std::string rawLine;
		while (std::getline(is, rawLine)) {
			std::string line = Trim(rawLine);
			if (line.empty() || line[0] == '#') continue;

			if (line.rfind("BEGIN_POLYGON", 0) == 0 || line.rfind("POLYGON", 0) == 0) {
				currentEdges.clear();
				inPolygon = true;
				continue;
			}

			if (line.rfind("END_POLYGON", 0) == 0 || line.rfind("END", 0) == 0) {
				if (inPolygon && !currentEdges.empty()) {
					out.emplace_back(std::move(currentEdges));
					currentEdges.clear();
				}
				inPolygon = false;
				continue;
			}

			if (inPolygon) {
				auto tokens = SplitTokens(line);
				SerializedEdge<CoordType> edge;
				if (ParseEdge(tokens, edge))
					currentEdges.push_back(edge);
			}
		}

		// 处理文件末尾没有 END_POLYGON 的情况
		if (inPolygon && !currentEdges.empty())
			out.emplace_back(std::move(currentEdges));

		return !out.empty();
	}

	/**
	 * @brief 从文件读取多边形数据
	 */
	static bool ReadFile(const std::string& filepath, std::vector<SerializedPolygon<CoordType>>& out) {
		std::ifstream file(filepath);
		if (!file.is_open()) return false;
		return Read(file, out);
	}

	// ---- 写入 ----
	/**
	 * @brief 将多边形数据写入输出流
	 * @param os    输出流
	 * @param polys 多边形列表
	 * @param comment 文件头注释（可选）
	 */
	static bool Write(std::ostream& os,
		const std::vector<SerializedPolygon<CoordType>>& polys,
		const std::string& comment = "Polygon Data File") {
		os << "# " << comment << "\n";
		os << "# Format: startX startY endX endY bulge\n";
		os << "# bulge: 0 = line, non-zero = arc\n\n";

		for (size_t pi = 0; pi < polys.size(); ++pi) {
			os << "BEGIN_POLYGON " << (pi + 1) << "\n";
			for (const auto& e : polys[pi]) {
				os << e.sx << " " << e.sy << " "
					<< e.ex << " " << e.ey << " "
					<< e.bulge << "\n";
			}
			os << "END_POLYGON\n\n";
		}
		return true;
	}

	/**
	 * @brief 将多边形数据写入文件
	 */
	static bool WriteFile(const std::string& filepath,
		const std::vector<SerializedPolygon<CoordType>>& polys,
		const std::string& comment = "Polygon Data File") {
		std::ofstream file(filepath);
		if (!file.is_open()) return false;
		return Write(file, polys, comment);
	}
};

// ============================================================================
// 二进制格式 IO
// ============================================================================

/** 二进制格式常量 */
namespace binary {
constexpr uint32_t MAGIC = 0x52504C54; // "TLPR" (little-endian)
constexpr uint8_t  VERSION = 1;

enum class Flag : uint8_t {
	None = 0,
	HasBulge = 1 << 0,   // 每条边包含 bulge 字段
	HasUserData = 1 << 1,   // 每条边包含用户数据段
};

inline Flag operator|(Flag a, Flag b) {
	return static_cast<Flag>(static_cast<uint8_t>(a) | static_cast<uint8_t>(b));
}
inline bool HasFlag(uint8_t flags, Flag f) {
	return (flags & static_cast<uint8_t>(f)) != 0;
}
} // namespace binary

/**
 * @brief 二进制格式多边形读写器
 * @tparam CoordType  坐标数值类型
 */
template <typename CoordType = double>
class BinaryPolygonIO {
public:
	// ---- 读取 ----
	/**
	 * @brief 从输入流读取二进制多边形数据
	 * @param is  输入流（需以 binary 模式打开）
	 * @param out 输出多边形列表
	 * @return 是否成功
	 */
	static bool Read(std::istream& is, std::vector<SerializedPolygon<CoordType>>& out) {
		out.clear();

		// 读取并校验头
		uint32_t magic = 0;
		uint8_t  version = 0;
		uint8_t  flags = 0;
		uint16_t reserved = 0;

		if (!ReadRaw(is, magic))   return false;
		if (!ReadRaw(is, version)) return false;
		if (!ReadRaw(is, flags))   return false;
		if (!ReadRaw(is, reserved)) return false;
		(void)reserved;

		if (magic != binary::MAGIC)   return false;
		if (version != binary::VERSION) return false;

		bool hasBulge = binary::HasFlag(flags, binary::Flag::HasBulge);
		bool hasUserData = binary::HasFlag(flags, binary::Flag::HasUserData);

		// 读取多边形数量
		uint32_t polyCount = 0;
		if (!ReadRaw(is, polyCount)) return false;

		out.reserve(polyCount);
		for (uint32_t pi = 0; pi < polyCount; ++pi) {
			uint32_t edgeCount = 0;
			if (!ReadRaw(is, edgeCount)) return false;

			SerializedPolygon<CoordType> poly;
			poly.reserve(edgeCount);

			for (uint32_t ei = 0; ei < edgeCount; ++ei) {
				SerializedEdge<CoordType> edge{};
				if (!ReadRaw(is, edge.sx)) return false;
				if (!ReadRaw(is, edge.sy)) return false;
				if (!ReadRaw(is, edge.ex)) return false;
				if (!ReadRaw(is, edge.ey)) return false;

				if (hasBulge) {
					if (!ReadRaw(is, edge.bulge)) return false;
				}

				if (hasUserData) {
					uint32_t dataSize = 0;
					if (!ReadRaw(is, dataSize)) return false;
					if (dataSize > 0)
						is.ignore(static_cast<std::streamsize>(dataSize));
				}

				poly.push_back(edge);
			}
			out.emplace_back(std::move(poly));
		}

		return true;
	}

	/**
	 * @brief 从文件读取二进制多边形数据
	 */
	static bool ReadFile(const std::string& filepath, std::vector<SerializedPolygon<CoordType>>& out) {
		std::ifstream file(filepath, std::ios::binary);
		if (!file.is_open()) return false;
		return Read(file, out);
	}

	// ---- 写入 ----
	/**
	 * @brief 将多边形数据以二进制格式写入输出流
	 * @param os        输出流（需以 binary 模式打开）
	 * @param polys     多边形列表
	 * @param hasBulge  是否写入 bulge 字段
	 * @param hasUserData 是否写入用户数据段（此时每条边后需紧跟 userDataSize + userData）
	 */
	static bool Write(std::ostream& os,
		const std::vector<SerializedPolygon<CoordType>>& polys,
		bool hasBulge = true,
		bool hasUserData = false) {
		// 写入头
		uint8_t flags = static_cast<uint8_t>(binary::Flag::None);
		if (hasBulge)    flags = static_cast<uint8_t>(flags | static_cast<uint8_t>(binary::Flag::HasBulge));
		if (hasUserData) flags = static_cast<uint8_t>(flags | static_cast<uint8_t>(binary::Flag::HasUserData));

		if (!WriteRaw(os, binary::MAGIC))   return false;
		if (!WriteRaw(os, binary::VERSION)) return false;
		if (!WriteRaw(os, flags))            return false;
		uint16_t reserved = 0;
		if (!WriteRaw(os, reserved))         return false;

		// 写入多边形数量
		auto polyCount = static_cast<uint32_t>(polys.size());
		if (!WriteRaw(os, polyCount)) return false;

		for (const auto& poly : polys) {
			auto edgeCount = static_cast<uint32_t>(poly.size());
			if (!WriteRaw(os, edgeCount)) return false;

			for (const auto& e : poly) {
				if (!WriteRaw(os, e.sx)) return false;
				if (!WriteRaw(os, e.sy)) return false;
				if (!WriteRaw(os, e.ex)) return false;
				if (!WriteRaw(os, e.ey)) return false;

				if (hasBulge) {
					if (!WriteRaw(os, e.bulge)) return false;
				}
				// NOTE: hasUserData 在此函数中未处理每条边的用户数据；
				//       如需每条边携带用户数据，请使用带 EdgeTraits 的模板重载
			}
		}

		return true;
	}

	/**
	 * @brief 将多边形数据以二进制格式写入文件
	 */
	static bool WriteFile(const std::string& filepath,
		const std::vector<SerializedPolygon<CoordType>>& polys,
		bool hasBulge = true,
		bool hasUserData = false) {
		std::ofstream file(filepath, std::ios::binary);
		if (!file.is_open()) return false;
		return Write(file, polys, hasBulge, hasUserData);
	}

	// ---- POD 读写辅助（公开，供自由函数使用） ----
	template <typename T>
	static bool ReadRaw(std::istream& is, T& val) {
		static_assert(std::is_trivially_copyable_v<T>);
		return !!is.read(reinterpret_cast<char*>(&val), sizeof(T));
	}

	template <typename T>
	static bool WriteRaw(std::ostream& os, const T& val) {
		static_assert(std::is_trivially_copyable_v<T>);
		return !!os.write(reinterpret_cast<const char*>(&val), sizeof(T));
	}
};

// ============================================================================
// 用户数据感知的二进制读写（模板版本，使用 EdgeDataTraits）
// ============================================================================

/**
 * @brief 从原始边列表中提取 SerializedPolygon 并写入二进制流
 * @tparam Edge      边类型
 * @tparam Traits    边数据萃取器
 *
 * 自动处理 bulge 和用户数据。
 */
template <typename Edge, typename Traits = EdgeDataTraits<Edge>>
bool WriteEdgesBinary(std::ostream& os, const std::vector<std::vector<Edge>>& polygons) {
	using CoordType = typename Traits::CoordType;

	constexpr bool hasBulge = Traits::HasBulge();
	constexpr bool hasUserData = Traits::HasUserData();

	uint8_t flags = static_cast<uint8_t>(binary::Flag::None);
	if constexpr (hasBulge)    flags = static_cast<uint8_t>(static_cast<uint8_t>(flags) | static_cast<uint8_t>(binary::Flag::HasBulge));
	if constexpr (hasUserData) flags = static_cast<uint8_t>(static_cast<uint8_t>(flags) | static_cast<uint8_t>(binary::Flag::HasUserData));

	// Header
	BinaryPolygonIO<CoordType>::WriteRaw(os, binary::MAGIC);
	BinaryPolygonIO<CoordType>::WriteRaw(os, binary::VERSION);
	BinaryPolygonIO<CoordType>::WriteRaw(os, flags);
	uint16_t reserved = 0;
	BinaryPolygonIO<CoordType>::WriteRaw(os, reserved);

	// Polygon count
	auto polyCount = static_cast<uint32_t>(polygons.size());
	BinaryPolygonIO<CoordType>::WriteRaw(os, polyCount);

	for (const auto& poly : polygons) {
		auto edgeCount = static_cast<uint32_t>(poly.size());
		BinaryPolygonIO<CoordType>::WriteRaw(os, edgeCount);

		for (const auto& e : poly) {
			CoordType sx = Traits::GetStartX(e);
			CoordType sy = Traits::GetStartY(e);
			CoordType ex = Traits::GetEndX(e);
			CoordType ey = Traits::GetEndY(e);

			BinaryPolygonIO<CoordType>::WriteRaw(os, sx);
			BinaryPolygonIO<CoordType>::WriteRaw(os, sy);
			BinaryPolygonIO<CoordType>::WriteRaw(os, ex);
			BinaryPolygonIO<CoordType>::WriteRaw(os, ey);

			if constexpr (hasBulge) {
				double bulge = Traits::GetBulge(e);
				BinaryPolygonIO<CoordType>::WriteRaw(os, bulge);
			}

			if constexpr (hasUserData) {
				auto buf = Traits::SerializeUserData(e);
				auto dataSize = static_cast<uint32_t>(buf.size());
				BinaryPolygonIO<CoordType>::WriteRaw(os, dataSize);
				if (dataSize > 0)
					os.write(reinterpret_cast<const char*>(buf.data()), static_cast<std::streamsize>(buf.size()));
			}
		}
	}

	return !!os;
}

/**
 * @brief 从二进制流中读取并还原为边列表
 * @tparam Edge      边类型
 * @tparam Traits    边数据萃取器
 */
template <typename Edge, typename Traits = EdgeDataTraits<Edge>>
bool ReadEdgesBinary(std::istream& is, std::vector<std::vector<Edge>>& out) {
	using CoordType = typename Traits::CoordType;

	out.clear();

	// Header
	uint32_t magic = 0;
	uint8_t  version = 0;
	uint8_t  flags = 0;
	uint16_t reserved = 0;

	if (!BinaryPolygonIO<CoordType>::ReadRaw(is, magic))   return false;
	if (!BinaryPolygonIO<CoordType>::ReadRaw(is, version)) return false;
	if (!BinaryPolygonIO<CoordType>::ReadRaw(is, flags))   return false;
	if (!BinaryPolygonIO<CoordType>::ReadRaw(is, reserved)) return false;
	(void)reserved;

	if (magic != binary::MAGIC)   return false;
	if (version != binary::VERSION) return false;

	bool hasBulge = binary::HasFlag(flags, binary::Flag::HasBulge);
	bool hasUserData = binary::HasFlag(flags, binary::Flag::HasUserData);

	uint32_t polyCount = 0;
	if (!BinaryPolygonIO<CoordType>::ReadRaw(is, polyCount)) return false;

	out.reserve(polyCount);
	for (uint32_t pi = 0; pi < polyCount; ++pi) {
		uint32_t edgeCount = 0;
		if (!BinaryPolygonIO<CoordType>::ReadRaw(is, edgeCount)) return false;

		std::vector<Edge> poly;
		poly.reserve(edgeCount);

		for (uint32_t ei = 0; ei < edgeCount; ++ei) {
			CoordType sx{}, sy{}, ex{}, ey{};
			double    bulge = 0.0;
			std::vector<uint8_t> userData;

			if (!BinaryPolygonIO<CoordType>::ReadRaw(is, sx)) return false;
			if (!BinaryPolygonIO<CoordType>::ReadRaw(is, sy)) return false;
			if (!BinaryPolygonIO<CoordType>::ReadRaw(is, ex)) return false;
			if (!BinaryPolygonIO<CoordType>::ReadRaw(is, ey)) return false;

			if (hasBulge) {
				if (!BinaryPolygonIO<CoordType>::ReadRaw(is, bulge)) return false;
			}

			if (hasUserData) {
				uint32_t dataSize = 0;
				if (!BinaryPolygonIO<CoordType>::ReadRaw(is, dataSize)) return false;
				if (dataSize > 0) {
					userData.resize(dataSize);
					is.read(reinterpret_cast<char*>(userData.data()), static_cast<std::streamsize>(dataSize));
				}
			}

			poly.push_back(Traits::Construct(sx, sy, ex, ey, bulge, userData));
		}
		out.emplace_back(std::move(poly));
	}

	return true;
}

// ============================================================================
// 便捷接口：使用文件路径
// ============================================================================

template <typename Edge, typename Traits = EdgeDataTraits<Edge>>
bool WriteEdgesBinaryFile(const std::string& filepath, const std::vector<std::vector<Edge>>& polygons) {
	std::ofstream file(filepath, std::ios::binary);
	if (!file.is_open()) return false;
	return WriteEdgesBinary<Edge, Traits>(file, polygons);
}

template <typename Edge, typename Traits = EdgeDataTraits<Edge>>
bool ReadEdgesBinaryFile(const std::string& filepath, std::vector<std::vector<Edge>>& out) {
	std::ifstream file(filepath, std::ios::binary);
	if (!file.is_open()) return false;
	return ReadEdgesBinary<Edge, Traits>(file, out);
}

// ============================================================================
// 边类型转换
// ============================================================================

/**
 * @brief 将 SerializedPolygon 转换为指定边类型的列表
 * @tparam Edge   目标边类型
 * @tparam Traits 边数据萃取器
 */
template <typename Edge, typename CoordType, typename Traits = EdgeDataTraits<Edge>>
std::vector<Edge> ToEdges(const SerializedPolygon<CoordType>& sp) {
	std::vector<Edge> result;
	result.reserve(sp.size());
	for (const auto& se : sp) {
		result.push_back(Traits::Construct(
			se.sx, se.sy, se.ex, se.ey, se.bulge, {}));
	}
	return result;
}

/**
 * @brief 将指定边类型的列表转换为 SerializedPolygon
 * @tparam Edge   源边类型
 * @tparam Traits 边数据萃取器
 */
template <typename Edge, typename Traits = EdgeDataTraits<Edge>>
auto FromEdges(const std::vector<Edge>& edges)
-> SerializedPolygon<typename Traits::CoordType> {
	using CoordType = typename Traits::CoordType;
	SerializedPolygon<CoordType> result;
	result.reserve(edges.size());
	for (const auto& e : edges) {
		result.push_back({
			Traits::GetStartX(e),
			Traits::GetStartY(e),
			Traits::GetEndX(e),
			Traits::GetEndY(e),
			Traits::GetBulge(e)
			});
	}
	return result;
}

/**
 * @brief 将多个多边形的边列表批量转换为 SerializedPolygon 的容器
 */
template <typename Edge, typename Traits = EdgeDataTraits<Edge>>
std::vector<SerializedPolygon<typename Traits::CoordType>> FromEdgeGroups(
	const std::vector<std::vector<Edge>>& groups) {
	using CoordType = typename Traits::CoordType;
	std::vector<SerializedPolygon<CoordType>> result;
	result.reserve(groups.size());
	for (const auto& g : groups)
		result.push_back(FromEdges<Edge, Traits>(g));
	return result;
}

/**
 * @brief 将 SerializedPolygon 容器批量转换为边类型列表的容器
 */
template <typename Edge, typename CoordType, typename Traits = EdgeDataTraits<Edge>>
std::vector<std::vector<Edge>> ToEdgeGroups(const std::vector<SerializedPolygon<CoordType>>& sps) {
	std::vector<std::vector<Edge>> result;
	result.reserve(sps.size());
	for (const auto& sp : sps)
		result.push_back(ToEdges<Edge, CoordType, Traits>(sp));
	return result;
}

// ============================================================================
// 便捷统一接口（根据文件扩展名自动选择格式）
// ============================================================================

/**
 * @brief 将多边形保存到文件（根据扩展名自动选择文本或二进制格式）
 * @param filepath  文件路径（.txt 用文本, .bin 用二进制）
 * @param polygons  多边形列表
 */
template <typename CoordType = double>
bool Save(const std::string& filepath,
	const std::vector<SerializedPolygon<CoordType>>& polygons) {
	if (filepath.ends_with(".bin"))
		return BinaryPolygonIO<CoordType>::WriteFile(filepath, polygons);
	else
		return TextPolygonIO<CoordType>::WriteFile(filepath, polygons);
}

/**
 * @brief 从文件加载多边形（根据扩展名自动选择文本或二进制格式）
 * @param filepath  文件路径
 * @param out       输出多边形列表
 */
template <typename CoordType = double>
bool Load(const std::string& filepath,
	std::vector<SerializedPolygon<CoordType>>& out) {
	if (filepath.ends_with(".bin"))
		return BinaryPolygonIO<CoordType>::ReadFile(filepath, out);
	else
		return TextPolygonIO<CoordType>::ReadFile(filepath, out);
}

// ============================================================================
// 带边类型模板的便捷统一接口
// ============================================================================

/**
 * @brief 从文件中加载并转换为指定边类型
 * @tparam Edge   目标边类型
 * @tparam Traits 边数据萃取器
 */
template <typename Edge, typename CoordType = double,
	typename Traits = EdgeDataTraits<Edge>>
	bool LoadAs(const std::string& filepath, std::vector<std::vector<Edge>>& out) {
	std::vector<SerializedPolygon<CoordType>> sps;
	if (!Load<CoordType>(filepath, sps)) return false;
	out = ToEdgeGroups<Edge, CoordType, Traits>(sps);
	return !out.empty();
}

/**
 * @brief 将边列表保存到文件
 * @tparam Edge   边类型
 * @tparam Traits 边数据萃取器
 */
template <typename Edge, typename Traits = EdgeDataTraits<Edge>>
bool SaveAs(const std::string& filepath, const std::vector<std::vector<Edge>>& polygons) {
	if (filepath.ends_with(".bin")) {
		std::ofstream file(filepath, std::ios::binary);
		if (!file.is_open()) return false;
		return WriteEdgesBinary<Edge, Traits>(file, polygons);
	} else {
		auto sps = FromEdgeGroups<Edge, Traits>(polygons);
		return TextPolygonIO<typename Traits::CoordType>::WriteFile(filepath, sps);
	}
}

// ============================================================================
// Drafting（Tailor::Execute() 输出）的序列化/反序列化
// ============================================================================

/** Drafting 二进制格式常量 */
namespace drafting_binary {
constexpr uint32_t MAGIC = 0x46445254; // "TDRF" (Tailor DRaFting, little-endian)
constexpr uint8_t  VERSION = 1;

enum class Flag : uint8_t {
	None = 0,
	HasBulge = 1 << 0,
	HasUserData = 1 << 1,
	IsPolygonSetB = 1 << 2,
	Reversed = 1 << 3,
	Discarded = 1 << 4,
	End = 1 << 5,
	HasAggregatedEdges = 1 << 6,
};

inline bool HasFlag(uint8_t flags, Flag f) {
	return (flags & static_cast<uint8_t>(f)) != 0;
}
} // namespace drafting_binary

/**
 * @brief 将 Drafting（Tailor::Execute() 输出）以二进制格式写入流
 *
 * @tparam Edge   边类型
 * @tparam Traits 边数据萃取器（默认 EdgeDataTraits<Edge>）
 *
 * @param os            输出流（需以 binary 模式打开）
 * @param edgeEvents    EdgeEvent 列表
 * @param vertexEvents  TopoVertex 列表
 * @return 是否成功
 *
 * 序列化内容: 边的几何信息 + 拓扑标志(wind/reversed/group handle等) + 顶点组
 */
template <typename Edge, typename Traits = EdgeDataTraits<Edge>>
bool WriteDraftingBinary(std::ostream& os,
	const std::vector<EdgeEvent<Edge>>& edgeEvents,
	const std::vector<TopoVertex>& vertexEvents) {
	using CoordType = typename Traits::CoordType;

	// ---- 写入头 ----
	BinaryPolygonIO<CoordType>::WriteRaw(os, drafting_binary::MAGIC);
	BinaryPolygonIO<CoordType>::WriteRaw(os, drafting_binary::VERSION);
	uint8_t reservedFlags = 0;
	BinaryPolygonIO<CoordType>::WriteRaw(os, reservedFlags);
	uint16_t reserved = 0;
	BinaryPolygonIO<CoordType>::WriteRaw(os, reserved);

	// ---- 写入 EdgeEvent 列表 ----
	auto edgeCount = static_cast<uint32_t>(edgeEvents.size());
	BinaryPolygonIO<CoordType>::WriteRaw(os, edgeCount);

	for (const auto& ee : edgeEvents) {
		// 构造 per-edge flags 字节
		uint8_t flags = static_cast<uint8_t>(drafting_binary::Flag::None);

		constexpr bool hasBulge = Traits::HasBulge();
		constexpr bool hasUserData = Traits::HasUserData();

		if constexpr (hasBulge)
			flags |= static_cast<uint8_t>(drafting_binary::Flag::HasBulge);
		if constexpr (hasUserData)
			flags |= static_cast<uint8_t>(drafting_binary::Flag::HasUserData);
		if (ee.isPolygonSetB)
			flags |= static_cast<uint8_t>(drafting_binary::Flag::IsPolygonSetB);
		if (ee.reversed)
			flags |= static_cast<uint8_t>(drafting_binary::Flag::Reversed);
		if (ee.discarded)
			flags |= static_cast<uint8_t>(drafting_binary::Flag::Discarded);
		if (ee.end)
			flags |= static_cast<uint8_t>(drafting_binary::Flag::End);
		if (ee.aggregatedEdges)
			flags |= static_cast<uint8_t>(drafting_binary::Flag::HasAggregatedEdges);

		BinaryPolygonIO<CoordType>::WriteRaw(os, flags);

		// 几何数据
		CoordType sx = Traits::GetStartX(ee.edge);
		CoordType sy = Traits::GetStartY(ee.edge);
		CoordType ex = Traits::GetEndX(ee.edge);
		CoordType ey = Traits::GetEndY(ee.edge);
		BinaryPolygonIO<CoordType>::WriteRaw(os, sx);
		BinaryPolygonIO<CoordType>::WriteRaw(os, sy);
		BinaryPolygonIO<CoordType>::WriteRaw(os, ex);
		BinaryPolygonIO<CoordType>::WriteRaw(os, ey);

		if constexpr (hasBulge) {
			double bulge = Traits::GetBulge(ee.edge);
			BinaryPolygonIO<CoordType>::WriteRaw(os, bulge);
		}

		if constexpr (hasUserData) {
			auto buf = Traits::SerializeUserData(ee.edge);
			auto dataSize = static_cast<uint32_t>(buf.size());
			BinaryPolygonIO<CoordType>::WriteRaw(os, dataSize);
			if (dataSize > 0)
				os.write(reinterpret_cast<const char*>(buf.data()), static_cast<std::streamsize>(buf.size()));
		}

		// 拓扑/环绕数数据
		auto windB = static_cast<int32_t>(ee.windB);
		auto windA = static_cast<int32_t>(ee.windA);
		BinaryPolygonIO<CoordType>::WriteRaw(os, windB);
		BinaryPolygonIO<CoordType>::WriteRaw(os, windA);
		BinaryPolygonIO<CoordType>::WriteRaw(os, ee.startPntGroup);
		BinaryPolygonIO<CoordType>::WriteRaw(os, ee.endPntGroup);
		BinaryPolygonIO<CoordType>::WriteRaw(os, ee.firstSplit);
		BinaryPolygonIO<CoordType>::WriteRaw(os, ee.secondSplit);
		BinaryPolygonIO<CoordType>::WriteRaw(os, ee.firstMerge);
		BinaryPolygonIO<CoordType>::WriteRaw(os, ee.firstBottom);
		BinaryPolygonIO<CoordType>::WriteRaw(os, ee.source);

		// 聚合边
		if (ee.aggregatedEdges) {
			auto aggrCount = static_cast<uint32_t>(ee.aggregatedEdges->sourceEdges.size());
			BinaryPolygonIO<CoordType>::WriteRaw(os, aggrCount);
			for (auto h : ee.aggregatedEdges->sourceEdges)
				BinaryPolygonIO<CoordType>::WriteRaw(os, h);
		}
	}

	// ---- 写入 TopoVertex 列表 ----
	auto vertexCount = static_cast<uint32_t>(vertexEvents.size());
	BinaryPolygonIO<CoordType>::WriteRaw(os, vertexCount);

	for (const auto& tv : vertexEvents) {
		// startGroup
		auto startSpan = tv.startGroup.Span();
		auto startCount = static_cast<uint32_t>(startSpan.size());
		BinaryPolygonIO<CoordType>::WriteRaw(os, startCount);
		for (auto h : startSpan)
			BinaryPolygonIO<CoordType>::WriteRaw(os, h);

		// endGroup
		auto endSpan = tv.endGroup.Span();
		auto endCount = static_cast<uint32_t>(endSpan.size());
		BinaryPolygonIO<CoordType>::WriteRaw(os, endCount);
		for (auto h : endSpan)
			BinaryPolygonIO<CoordType>::WriteRaw(os, h);
	}

	return !!os;
}

/**
 * @brief 从二进制流中还原 Drafting（Tailor::Execute() 输出）
 *
 * @tparam Edge   边类型
 * @tparam Traits 边数据萃取器（默认 EdgeDataTraits<Edge>）
 *
 * @param is            输入流（需以 binary 模式打开）
 * @param edgeEvents    输出 EdgeEvent 列表
 * @param vertexEvents  输出 TopoVertex 列表
 * @return 是否成功
 */
template <typename Edge, typename Traits = EdgeDataTraits<Edge>>
bool ReadDraftingBinary(std::istream& is,
	std::vector<EdgeEvent<Edge>>& edgeEvents,
	std::vector<TopoVertex>& vertexEvents) {
	using CoordType = typename Traits::CoordType;

	edgeEvents.clear();
	vertexEvents.clear();

	// ---- 读取并校验头 ----
	uint32_t magic = 0;
	uint8_t  version = 0;
	uint8_t  flagsReserved = 0;
	uint16_t reserved = 0;

	if (!BinaryPolygonIO<CoordType>::ReadRaw(is, magic))   return false;
	if (!BinaryPolygonIO<CoordType>::ReadRaw(is, version)) return false;
	if (!BinaryPolygonIO<CoordType>::ReadRaw(is, flagsReserved)) return false;
	if (!BinaryPolygonIO<CoordType>::ReadRaw(is, reserved)) return false;

	if (magic != drafting_binary::MAGIC)     return false;
	if (version != drafting_binary::VERSION) return false;
	(void)flagsReserved;
	(void)reserved;

	constexpr bool hasBulge = Traits::HasBulge();
	constexpr bool hasUserData = Traits::HasUserData();

	// ---- 读取 EdgeEvent 列表 ----
	uint32_t edgeCount = 0;
	if (!BinaryPolygonIO<CoordType>::ReadRaw(is, edgeCount)) return false;

	edgeEvents.reserve(edgeCount);
	for (uint32_t i = 0; i < edgeCount; ++i) {
		uint8_t flags = 0;
		if (!BinaryPolygonIO<CoordType>::ReadRaw(is, flags)) return false;

		bool flagHasBulge = drafting_binary::HasFlag(flags, drafting_binary::Flag::HasBulge);
		bool flagHasUserData = drafting_binary::HasFlag(flags, drafting_binary::Flag::HasUserData);
		bool isPolygonSetB = drafting_binary::HasFlag(flags, drafting_binary::Flag::IsPolygonSetB);
		bool reversed = drafting_binary::HasFlag(flags, drafting_binary::Flag::Reversed);
		bool discarded = drafting_binary::HasFlag(flags, drafting_binary::Flag::Discarded);
		bool ended = drafting_binary::HasFlag(flags, drafting_binary::Flag::End);
		bool hasAggregated = drafting_binary::HasFlag(flags, drafting_binary::Flag::HasAggregatedEdges);

		// 几何数据
		CoordType sx{}, sy{}, ex{}, ey{};
		double    bulge = 0.0;
		std::vector<uint8_t> userData;

		if (!BinaryPolygonIO<CoordType>::ReadRaw(is, sx)) return false;
		if (!BinaryPolygonIO<CoordType>::ReadRaw(is, sy)) return false;
		if (!BinaryPolygonIO<CoordType>::ReadRaw(is, ex)) return false;
		if (!BinaryPolygonIO<CoordType>::ReadRaw(is, ey)) return false;

		if (flagHasBulge) {
			if (!BinaryPolygonIO<CoordType>::ReadRaw(is, bulge)) return false;
		}

		if (flagHasUserData) {
			uint32_t dataSize = 0;
			if (!BinaryPolygonIO<CoordType>::ReadRaw(is, dataSize)) return false;
			if (dataSize > 0) {
				userData.resize(dataSize);
				is.read(reinterpret_cast<char*>(userData.data()), static_cast<std::streamsize>(dataSize));
			}
		}

		// 拓扑/环绕数数据
		int32_t windB = 0, windA = 0;
		Handle startPntGroup = npos;
		Handle endPntGroup = npos;
		Handle firstSplitH = npos;
		Handle secondSplitH = npos;
		Handle firstMergeH = npos;
		Handle firstBottomH = npos;
		Handle sourceH = npos;

		if (!BinaryPolygonIO<CoordType>::ReadRaw(is, windB)) return false;
		if (!BinaryPolygonIO<CoordType>::ReadRaw(is, windA)) return false;
		if (!BinaryPolygonIO<CoordType>::ReadRaw(is, startPntGroup)) return false;
		if (!BinaryPolygonIO<CoordType>::ReadRaw(is, endPntGroup)) return false;
		if (!BinaryPolygonIO<CoordType>::ReadRaw(is, firstSplitH)) return false;
		if (!BinaryPolygonIO<CoordType>::ReadRaw(is, secondSplitH)) return false;
		if (!BinaryPolygonIO<CoordType>::ReadRaw(is, firstMergeH)) return false;
		if (!BinaryPolygonIO<CoordType>::ReadRaw(is, firstBottomH)) return false;
		if (!BinaryPolygonIO<CoordType>::ReadRaw(is, sourceH)) return false;

		// 聚合边
		std::unique_ptr<AggregatedEdgeEvent> aggregated;
		if (hasAggregated) {
			uint32_t aggrCount = 0;
			if (!BinaryPolygonIO<CoordType>::ReadRaw(is, aggrCount)) return false;
			aggregated = std::make_unique<AggregatedEdgeEvent>();
			aggregated->sourceEdges.reserve(aggrCount);
			for (uint32_t ai = 0; ai < aggrCount; ++ai) {
				Handle h = npos;
				if (!BinaryPolygonIO<CoordType>::ReadRaw(is, h)) return false;
				aggregated->sourceEdges.push_back(h);
			}
		}

		// 构造 EdgeEvent
		EdgeEvent<Edge> ee{ Traits::Construct(sx, sy, ex, ey, bulge, userData) };
		ee.id = i;
		ee.isPolygonSetB = isPolygonSetB;
		ee.reversed = reversed;
		ee.discarded = discarded;
		ee.end = ended;
		ee.windB = static_cast<Int>(windB);
		ee.windA = static_cast<Int>(windA);
		ee.startPntGroup = startPntGroup;
		ee.endPntGroup = endPntGroup;
		ee.firstSplit = firstSplitH;
		ee.secondSplit = secondSplitH;
		ee.firstMerge = firstMergeH;
		ee.firstBottom = firstBottomH;
		ee.source = sourceH;
		ee.aggregatedEdges = std::move(aggregated);

		edgeEvents.push_back(std::move(ee));
	}

	// ---- 读取 TopoVertex 列表 ----
	uint32_t vertexCount = 0;
	if (!BinaryPolygonIO<CoordType>::ReadRaw(is, vertexCount)) return false;

	vertexEvents.reserve(vertexCount);
	for (uint32_t i = 0; i < vertexCount; ++i) {
		TopoVertex tv;
		tv.id = i;

		// startGroup
		{
			uint32_t count = 0;
			if (!BinaryPolygonIO<CoordType>::ReadRaw(is, count)) return false;
			for (uint32_t j = 0; j < count; ++j) {
				Handle h = npos;
				if (!BinaryPolygonIO<CoordType>::ReadRaw(is, h)) return false;
				tv.startGroup.Insert(h);
			}
		}

		// endGroup
		{
			uint32_t count = 0;
			if (!BinaryPolygonIO<CoordType>::ReadRaw(is, count)) return false;
			for (uint32_t j = 0; j < count; ++j) {
				Handle h = npos;
				if (!BinaryPolygonIO<CoordType>::ReadRaw(is, h)) return false;
				tv.endGroup.Insert(h);
			}
		}

		vertexEvents.push_back(std::move(tv));
	}

	return true;
}

// ---- 文件路径便捷包装 ----

template <typename Edge, typename Traits = EdgeDataTraits<Edge>>
bool WriteDraftingFile(const std::string& filepath,
	const std::vector<EdgeEvent<Edge>>& edgeEvents,
	const std::vector<TopoVertex>& vertexEvents) {
	std::ofstream file(filepath, std::ios::binary);
	if (!file.is_open()) return false;
	return WriteDraftingBinary<Edge, Traits>(file, edgeEvents, vertexEvents);
}

template <typename Edge, typename Traits = EdgeDataTraits<Edge>>
bool ReadDraftingFile(const std::string& filepath,
	std::vector<EdgeEvent<Edge>>& edgeEvents,
	std::vector<TopoVertex>& vertexEvents) {
	std::ifstream file(filepath, std::ios::binary);
	if (!file.is_open()) return false;
	return ReadDraftingBinary<Edge, Traits>(file, edgeEvents, vertexEvents);
}
} // namespace polygon_io

TAILOR_NAMESPACE_END
