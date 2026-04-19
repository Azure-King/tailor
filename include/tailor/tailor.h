#pragma once

#include "tailor_concept.h"
#include <set>
#include <queue>
#include <algorithm>
#include <assert.h>
#include <variant> // C++17
#include <functional>
#include <memory>
#include <array>
#include <vector>
#include <numeric>
#include <span> // C++17

TAILOR_NAMESPACE_BEGIN

namespace {
bool Positive(VertexRelativePositionType type) {
	return static_cast<int>(type) > 0;
}
bool Negative(VertexRelativePositionType type) {
	return static_cast<int>(type) < 0;
}

// from boost
inline void HashCombine(size_t& seed, size_t value) {
	seed ^= value + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}
}

struct VertexEventGroup {
	static constexpr size_t MAX_VERTEX_SIZE = 4;
	VertexEventGroup() = default;
	VertexEventGroup(Handle edge_a, bool is_start_a) {
		edges[0] = edge_a; isStart[0] = is_start_a;
	}

	VertexEventGroup(
		Handle edge_a, bool is_start_a,
		Handle edge_b, bool is_start_b) {
		edges[0] = edge_a; isStart[0] = is_start_a;
		edges[1] = edge_b; isStart[1] = is_start_b;
	}
	VertexEventGroup(
		Handle edge_a, bool is_start_a,
		Handle edge_b, bool is_start_b,
		Handle edge_c, bool is_start_c) {
		edges[0] = edge_a; isStart[0] = is_start_a;
		edges[1] = edge_b; isStart[1] = is_start_b;
		edges[2] = edge_c; isStart[2] = is_start_c;
	}
	VertexEventGroup(
		Handle edge_a, bool is_start_a,
		Handle edge_b, bool is_start_b,
		Handle edge_c, bool is_start_c,
		Handle edge_d, bool is_start_d) {
		edges[0] = edge_a; isStart[0] = is_start_a;
		edges[1] = edge_b; isStart[1] = is_start_b;
		edges[2] = edge_c; isStart[2] = is_start_c;
		edges[3] = edge_d; isStart[3] = is_start_d;
	}

	void Add(Handle edge_a, bool is_start_a) {
		assert(edges[MAX_VERTEX_SIZE - 1] == npos); // 必须要求空间装入
		for (size_t i = 0; i < MAX_VERTEX_SIZE; ++i) {
			if (edges[i] != npos) continue;
			edges[i] = edge_a; isStart[i] = is_start_a;
			return;
		}
	}

	void Add(Handle edge_a, bool is_start_a, Handle edge_b, bool is_start_b) {
		assert(edges[MAX_VERTEX_SIZE - 1] == npos); // 必须要求空间装入
		for (size_t i = 0; i < MAX_VERTEX_SIZE - 1; ++i) {
			if (edges[i] != npos) continue;
			edges[i] = edge_a; isStart[i] = is_start_a;
			edges[i + 1] = edge_b; isStart[i + 1] = is_start_b;
			return;
		}
	}

	Handle edges[MAX_VERTEX_SIZE]{ npos,npos,npos,npos };
	bool isStart[4]{};
};

class EdgeGroup {
private:
	using DArray = std::vector<Handle>;
	static constexpr size_t STATIC_ARRAY_SIZE = sizeof(DArray) / sizeof(Handle);
	using SArray = std::array<Handle, STATIC_ARRAY_SIZE>;
	std::variant<SArray, DArray> array = MakeEmptyStaticArray();

	constexpr SArray MakeEmptyStaticArray() const {
		SArray arr{};
		arr.fill(tailor::npos);
		return arr;
	}

	template<typename EdgeEvent>
	struct HandleReplacer {
		Handle eid;
		const std::vector<EdgeEvent>& edgeEvents;

		template<typename Array>
		bool operator()(Array& arr) const {
			Handle target = eid;
			while (npos != target) {
				for (auto& id : arr) {
					if (npos == id) break;
					if (target != id) continue;

					// 替换
					id = eid;
					return true;
				}
				target = edgeEvents[target].source;
			}
			return false;
		}
	};

	struct HandleInserter {
		Handle eid;

		bool operator()(SArray& arr) const {
			// 按顺序寻找空位插入
			for (auto& id : arr) {
				if (npos != id) continue;
				id = eid;
				return true;
			}
			return false;
		}

		bool operator()(DArray& arr) const {
			arr.emplace_back(eid);
			return true;
		}
	};

	struct SizeCounter {
		size_t operator()(const SArray& arr) const {
			size_t count = 0;
			// 按顺序寻找空位插入
			for (auto& id : arr) {
				if (npos == id) break;
				++count;
			}
			return count;
		}

		size_t operator()(const DArray& arr) const {
			return arr.size();
		}
	};

	template<class Condition>
	struct EleVisitor {
		Condition condition;

		void operator()(const SArray& arr) const {
			for (auto& id : arr) {
				if (npos == id) break;
				condition(id);
			}
		}

		void operator()(const DArray& arr) const {
			for (auto id : arr) {
				condition(id);
			}
		}
	};

	struct SpanVisitor {
		std::span<const Handle> operator()(const SArray& arr) const {
			size_t count = 0;
			for (auto& id : arr) {
				if (npos == id) break;
				++count;
			}
			return std::span<const Handle>(arr.data(), count);
		}
		std::span<const Handle> operator()(const DArray& arr) const {
			return std::span<const Handle>(arr);
		}
	};
public:
	void Insert(Handle eid) {
		bool success = std::visit(HandleInserter{ eid }, array);
		if (success) return;

		// 处插入失败, 需要手动扩展数组
		auto& old_array = std::get<SArray>(array);
		std::vector<Handle> new_array;
		new_array.reserve(STATIC_ARRAY_SIZE * 2);
		new_array.insert(new_array.end(), old_array.begin(), old_array.end());
		new_array.emplace_back(eid);
	}

	/**
	 * @brief 将组中记录的点对应边(该边可能被废弃)替换为结果边
	 * @tparam EdgeEvent
	 * @param ended_event_id	结束的事件边索引
	 * @param edgeEvents		事件集合
	 * @return					是否成功替换
	 */
	template<class EdgeEvent>
	bool ReplaceEvent(Handle ended_event_id, const std::vector<EdgeEvent>& edge_events) {
		assert(edge_events[ended_event_id].end);
		return std::visit(HandleReplacer<EdgeEvent>{ ended_event_id, edge_events }, array);
	}

	template<class Condition>
	void Foreach(Condition&& fun) const {
		std::visit(EleVisitor<Condition>{std::forward<Condition>(fun)}, array);
	}

	size_t Size() const {
		return std::visit(SizeCounter{}, array);
	}

	auto begin() const {
		return std::visit([](const auto& arr) { return arr.data(); }, array);
	}
	auto end() const {
		return std::visit([this](const auto& arr) { return arr.data() + this->Size(); }, array);
	}

	std::span<const Handle> Span() const {
		return std::visit(SpanVisitor{}, array);
	}
};

struct TopoVertex {
	Handle id = npos;
	EdgeGroup startGroup; // 起点为该点的边集合
	EdgeGroup endGroup;	 // 终点为该点的边集合
};

// 聚合属性仅记录事件的 Handle, 但可访问属性仅为: isClipper, reversed, 用来计算 clipperWind 与 subjectWind
struct AggregatedEdgeEvent {
	std::vector<Handle> sourceEdges; // 源边的索引
};

template<EdgeConcept Edge>
struct EdgeEvent {
	Edge edge;						// 边
	Handle id = npos;				// 该边的索引(此属性其实是多余的, 但是为了考虑方便编程, 还是加上了)

	bool isClipper = false;			// 该边的组别
	bool reversed = false;			// 该边的方向是否与输入时相反, 即 monotonicity < 0
	bool discarded = false;			// 该边是否被废弃, 如果该边被废弃, 则一定会合并或分裂成其他事件, 见下方 firstMerge, firstSplit, secondSplit
	bool end = false;				// 该边事件是否为完全处理完毕, 当且仅当为 true 时, 可以作为输出事件

	// 边的单调性, 用于包围盒计算
	VertexRelativePositionType monotonicity = VertexRelativePositionType::Same /*非法值 */;

	Handle startPntGroup = npos;    // 起点所在顶点的索引, 用于建立点的拓扑
	Handle endPntGroup = npos;	    // 终点所在顶点的索引, 用于建立点的拓扑

	// source 每次可能被拆分成至多两条边:
	// 仅有相交: [firstSplit, secondSplit]
	// 部分重合: [firstMerge, secondSplit], 其中 firstMerge 与 source 的边实际输入方向可能并不相同, 需要在 firstMerge 内的 aggregatedEdges 中查询
	// 整条重合: [firstMerge], 其中 secondSplit 值为 npos
	Handle firstMerge = npos;		// 如果该边与其他边重合, 融合后的聚合边的索引
	Handle firstSplit = npos;		// 如果该边被分割, 分割后的第一条边的索引, 用于聚合点更新
	Handle secondSplit = npos;		// 如果该边被分割, 分割后的第二条边的索引, 用于聚合点更新

	Handle firstBottom = npos;	    // 下方第一条边的索引(该边可能是 discarded), 该属性用于计算环绕数
	Handle source = npos;			// 源边的索引, 但是如果该边为聚合边, 则实际的源边有多个

	Int clipperWind = 0;			// 裁剪边的环绕数, 如果 discarded ,则该属性无效
	Int subjectWind = 0;			// 被裁剪边的环绕数, 如果 discarded ,则该属性无效

	// 如果该边为聚合边, 则该属性不为空, 聚合边内的所有边的属性不一定全是相同的, 比如: isClipper, reversed
	// 该属性用于记录所有边的源边的索引, 但可访问属性仅为: isClipper, reversed, 用于计算 clipperWind 与 subjectWind
	std::unique_ptr<AggregatedEdgeEvent> aggregatedEdges = nullptr;

	bool IsAggregatedEdge() const {
		return (bool)aggregatedEdges;
	}
};

struct VertexEvent {
	Handle e = npos;
	bool start = false;
};
using VEvent = VertexEvent;
using VEventGroup = VertexEventGroup;

template<EdgeConcept Edge, EdgeAnalyzerConcept<Edge> EdgeAnalyzer>
class Tailor {
public:
	using Vertex = edge_analysis_vertex_t<EdgeAnalyzer, Edge>;
	using EdgeEventType = EdgeEvent<Edge>;
	using VEventType = VertexEvent;

	template<class... Args>
	Tailor(Args&&... args) :ea(std::forward<Args>(args)...) {
	}

	decltype(auto) Analyzer() {
		return ea;
	}
	decltype(auto) Analyzer() const {
		return ea;
	}
private:
	std::vector<Edge> clipper;
	std::vector<Edge> subject;

	EdgeAnalyzer ea;
public:
	template<EdgeIteratorConcept<Edge> EdgeIterator>
	void AddClipper(EdgeIterator begin, EdgeIterator end) {
		clipper.insert(clipper.cend(), begin, end);
	}
	template<EdgeIteratorConcept<Edge> EdgeIterator>
	void AddSubject(EdgeIterator begin, EdgeIterator end) {
		subject.insert(subject.cend(), begin, end);
	}

	struct PatternDrafting {
		using EdgeEvent = ::tailor::EdgeEvent<Edge>;
		std::vector<EdgeEvent> edgeEvent;
		std::vector<TopoVertex> vertexEvents;
	};

	PatternDrafting Execute();

private:
};

#include "detail/tailor_detail.h"

TAILOR_NAMESPACE_END