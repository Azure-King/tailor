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

struct VertexEvent2 {
	static constexpr size_t MAX_VERTEX_SIZE = 4;
	VertexEvent2() = default;
	VertexEvent2(Handle edge_a, bool is_start_a) {
		edges[0] = edge_a; isStart[0] = is_start_a;
	}

	VertexEvent2(
		Handle edge_a, bool is_start_a,
		Handle edge_b, bool is_start_b) {
		edges[0] = edge_a; isStart[0] = is_start_a;
		edges[1] = edge_b; isStart[1] = is_start_b;
	}
	VertexEvent2(
		Handle edge_a, bool is_start_a,
		Handle edge_b, bool is_start_b,
		Handle edge_c, bool is_start_c) {
		edges[0] = edge_a; isStart[0] = is_start_a;
		edges[1] = edge_b; isStart[1] = is_start_b;
		edges[2] = edge_c; isStart[2] = is_start_c;
	}
	VertexEvent2(
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

template<EdgeConcept Edge, EdgeAnalyzerConcept<Edge> EdgeAnalyzer>
class Tailor {
public:
	using Vertex = edge_analysis_vertex_t<EdgeAnalyzer, Edge>;

	// 聚合属性仅记录事件的 Handle, 但可访问属性仅为: isClipper, reversed, 用来计算 clipperWind 与 subjectWind
	struct AggregatedEdgeEvent {
		std::vector<Handle> sourceEdges; // 源边的索引
	};

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
		//Vertex v;
		Handle e = npos;
		bool start = false;
	};
	using VEvent = VertexEvent;
	using VEventGroup = VertexEvent2;

	// 事件比较器
	struct VertexEventComparator {
		EdgeAnalyzer& ea;
		std::vector<EdgeEvent>& edgeEvents;

		bool operator()(const VEvent& a, const VEvent& b) {
			auto rp = ea.CalcateVertexRelativePosition(a.v, b.v);
			return static_cast<int>(rp) < 0;
		}
		bool operator()(VEventGroup& a, VEventGroup& b) {
			return static_cast<int>(RelativePosition(a, b)) < 0;
		}

		auto RelativePosition(VEventGroup& a, VEventGroup& b) {
			const auto& ap = GetPnt(a);
			const auto& bp = GetPnt(b);
			auto rp = ea.CalcateVertexRelativePosition(ap, bp);
			return rp;
		}

		Vertex GetPnt(VEventGroup& p, size_t i = 0) {
			assert(p.edges[i] != npos);

			const auto& edge_event = GetUpdatedEdge(p, i);
			return p.isStart[0] ?
				ea.Start(edge_event.edge) :
				ea.End(edge_event.edge);
		}

		/**
		 * @brief 获取更新后的边, 如果更新后的边依旧被废弃, 则说明该边被融合, 有其他的聚合边代替
		 * @param p 顶点组
		 * @param i	序号
		 * @return 获取更新后的边引用
		 */
		const EdgeEvent& GetUpdatedEdge(VEventGroup& p, size_t i = 0) const {
			assert(p.edges[i] != npos);
			const EdgeEvent* event = &edgeEvents[p.edges[i]];
			if (p.isStart[0]) {
				while (event->discarded && event->firstSplit != npos) {
					event = &edgeEvents[event->firstSplit];
					p.edges[i] = event->id;
				}
			} else {
				while (event->discarded && event->secondSplit != npos) {
					event = &edgeEvents[event->secondSplit];
					p.edges[i] = event->id;
				}
			}
			return *event;
		}
	private:
	};

	template<class... Args>
	Tailor(Args&&... args) :ea(std::forward<Args>(args)...) {
	}

	decltype(auto) Analyzer() {
		return ea;
	}
	decltype(auto) Analyzer() const {
		return ea;
	}

	std::function<void(const Edge&)> DebugFunc;
	std::function<void(const EdgeEvent&)> DebugFunc2;
	std::function<void(const EdgeEvent&, bool start)> DebugFunc3;

private:

	template<class Container = std::vector<VEventGroup>>
	class VertexEventQueue {
	public:
		VertexEventQueue(EdgeAnalyzer& ea, std::vector<EdgeEvent>& ees, Container&& container = Container()) :ea(ea),
			comparator(VertexEventComparator{ ea,ees }),
			queue(VertexEventComparator{ ea,ees }, std::forward<Container>(container)) {
		}

		template<class... Args>
		void Push(Args&&... args) {
			queue.emplace(std::forward<Args>(args)...);
		}

		decltype(auto) Top() const {
			return queue.top();
		}

		decltype(auto) Pop() {
			auto ve = queue.top();
			queue.pop();
			return ve;
		}
		bool Empty() const {
			return queue.empty();
		}

		void PopAllEvents(
			std::vector<VertexEvent>& start_events,
			std::vector<VertexEvent>& end_events) {
			if (Empty()) {
				return;
			}

			VEventGroup temp = Pop();
			while (true) {
				Emplace(start_events, end_events, temp);

				// check
				//std::set<Handle> hhhh;
				//for (auto& xx : start_events) {
				//	hhhh.insert(xx.e);
				//	assert(!comparator.edgeEvents[xx.e].discarded);
				//}
				//for (auto& xx : end_events) {
				//	hhhh.insert(xx.e);
				//	assert(!comparator.edgeEvents[xx.e].discarded);
				//}
				//assert(hhhh.size() == start_events.size() + end_events.size());

				if (Empty()) return;
				VEventGroup& top = const_cast<VEventGroup&>(Top());
				if (VertexRelativePositionType::Same !=
					comparator.RelativePosition(temp, top)) return;
				temp = Pop();
			}
		}

	public:
		EdgeAnalyzer& ea;
		VertexEventComparator comparator;
		std::priority_queue<VEventGroup, Container, VertexEventComparator> queue;// 事件优先队列
	private:

		void Emplace(std::vector<VertexEvent>& start_events, std::vector<VertexEvent>& end_events, VEventGroup& group) {
			for (size_t i = 0; i < VEventGroup::MAX_VERTEX_SIZE; ++i) {
				if (group.edges[i] == npos) return; // 聚合点约定按照顺序存储的, 遇到 npos 可以提前结束

				auto& events = group.isStart[i] ? start_events : end_events;
				const auto& edge_event = comparator.GetUpdatedEdge(group, i);
				if (edge_event.discarded) continue; // 如果更新后的边被废弃, 说明该边有其他的聚合边代替, 因此不需要加入事件集

				events.emplace_back(group.edges[i], group.isStart[i]);
				//events.emplace_back(VertexEvent{ group.edges[i], group.isStart[i] });
			}
		}
	};

	template<class VEQ>
	class EdgeStateSet {
	public:
		EdgeStateSet(VEQ& veq, std::vector<EdgeEvent>& edge_events, EdgeAnalyzer& ea,
			const std::vector<Edge>& clipper, const std::vector<Edge>& subject) :
			veq(veq), edgeEvents(edge_events), ea(ea) {
			//topoVertices.reserve(7000);
			//edgeEvents.reserve(25000);
			size_t edge_size = clipper.size() + subject.size();
			topoVertices.reserve(edge_size * 2.5);
			edgeEvents.reserve(edge_size * 2.5);

			CreateEvents(clipper, subject);
		}

		void PreprocessEvents(std::vector<VEvent>& start_events2, std::vector<VEvent>& end_events) {
			std::vector<VEvent>& start_events = cache;
			start_events.clear();

			// 将起点事件集按顺序排列
			for (const auto& ve : start_events2) {
				InsertVertexEvent(start_events, ve);
			}

			// 1. 在排序前识别所有的聚合边, 并替换对应事件
			// 2. 排序事件, 优先处理相对位置在下方的边
			// 3. 将事件放在活跃边中合适的位置, 求出上下交点(或重合边), 并选取合适的点, 分裂事件边

			// 此处处理一个特殊情况: 起点在某一个活跃边的上
			// 理论上, 此时终点事件集应当为空, 否则先前已经将该边裂解了
			if (!end_events.empty() || start_events.empty()) {
				std::swap(start_events, start_events2);
				return;
			}

			const auto& p = GetPointInVertexEvent(start_events.front());
			for (auto it = events.begin(); it != events.end(); ++it) {
				if (!ea.IsOnEdge(p, GetEdgeEvent(it->e).edge)) continue;

				decltype(auto) result = ea.SplitEdge(GetEdgeEvent(it->e).edge, p);

				// 该分支处理 EdgeRelativePosition 无效的情况, 因为所有起点事件的位置相同,
				// 所以该情况在本函数中应该仅会存在最多一次, 且仅需测试一次

				assert(result.HasPiece(AI));
				assert(result.HasPiece(IB));

				// 裂解 it 指向的边
				auto split_result = SplitEvent(it->e,
					result.GetPiece(AI),
					result.GetPiece(IB)
				);

				// 将 it 指向的起始点事件替换为 AI 的起始点事件
				*it = MakeStartVertexEvent(split_result.aiEvent.id);

				// 终点事件集中加入 AI 的终点事件, 以便后续将其从活跃边中移除
				end_events.emplace_back(MakeEndVertexEvent(split_result.aiEvent.id));

				// 队列中加入 IB 的终点事件
				//veq.Push(MakeEndVertexEvent(split_result.ibEvent.id));

				// 插入事件最后执行, 防止引用失效
				// 将 it 对应的起点事件替换为 IB 的起点事件
				InsertVertexEvent(start_events, MakeStartVertexEvent(split_result.ibEvent.id));

				// 理论上, 活跃边中至多仅可能存在一条边出现此情况
				break;
			}

			std::swap(start_events, start_events2);
		}

		void AcceptSameLocStartEvents(std::vector<VEvent>& start_events, TopoVertex& vertex) {
			// 此时起点事件已经排序完毕
			for (const auto& ve : start_events) {
				// 此时插入的边可能后续会被废弃, 但是在结束事件时, 会通过溯源机制, 换回正确的 id
				vertex.startGroup.Insert(ve.e);
				GetEdgeEvent(ve.e).startPntGroup = vertex.id;
			}

			// 二分插入
			auto events_begin = events.begin();
			auto events_end = events.end();
			for (auto& cur : start_events) {
				auto begin = events_begin;
				auto end = events_end;

				auto it = events.begin();
				while (begin < end) {
					// middle
					it = begin + std::distance(begin, end) / 2;
					auto& debug_e0 = GetEdgeEvent(it->e);
					auto& debug_e1 = GetEdgeEvent(cur.e);

					auto res = ea.CalcateEdgeRelativePosition(
						GetEdgeEvent(it->e).edge,
						GetEdgeEvent(cur.e).edge, // 新边必须放在后面
						OnlyFocusOnRelativePositionWithoutCoincidence{}
					);

					// 上方已经处理了 CI 不存在的情况, 此处应该不会再出现
					//assert(res.HasPiece(CI));

					if (CurveRelativePositionType::Upward == res.RelativePositionType()) {
						begin = it + 1;
					} else {
						end = it;
					}
				}
				it = begin;

				std::optional<CurveRelativePositionResult2<Edge>> lower_intersection;
				if (it != events.begin()) {
					lower_intersection = std::make_optional<CurveRelativePositionResult2<Edge>>(
						ea.CalcateEdgeRelativePosition(
							GetEdgeEvent((it - 1)->e).edge,
							GetEdgeEvent(cur.e).edge, // 新边必须放在后面
							OnlyFocusOnRelativePositionDetailsWithoutCoincidence{}
						)
					);
				}

				std::optional<CurveRelativePositionResult2<Edge>> upper_intersection;
				if (it != events.end()) {
					upper_intersection = std::make_optional<CurveRelativePositionResult2<Edge>>(
						ea.CalcateEdgeRelativePosition(
							GetEdgeEvent(it->e).edge,
							GetEdgeEvent(cur.e).edge, // 新边必须放在后面
							OnlyFocusOnRelativePositionDetailsWithoutCoincidence{}
						)
					);
				}

				if (!lower_intersection && !upper_intersection) {
					GroupWindResult{} >> GetEdgeEvent(cur.e);

					// 活跃边为空
					events_begin = events.insert(it, cur);
					events_end = events_begin + 1;

					continue;
				}

				bool use_upper = false;

				if (lower_intersection && !upper_intersection) {
					use_upper = false;
				} else if (!lower_intersection && upper_intersection) {
					use_upper = true;
				} else if (lower_intersection && upper_intersection) {
					bool lower_has_intersection = lower_intersection->HasPiece(IB) || lower_intersection->HasPiece(ID);
					bool upper_has_intersection = upper_intersection->HasPiece(IB) || upper_intersection->HasPiece(ID);

					if (!lower_has_intersection && !upper_has_intersection) {
						CalcGroupWind(events.begin(), it) >> GetEdgeEvent(cur.e);

						auto new_loc = events.emplace(it, cur);
						events_begin = new_loc;
						events_end = events_begin + 1;

						if (events.cbegin() != new_loc) {
							// 如果新加入的活跃边下方有别的活跃边, 则需要记录下方第一条边
							GetEdgeEvent(new_loc->e).firstBottom = (new_loc - 1)->e;
						}
						continue;
					} else if (!lower_has_intersection && upper_has_intersection) {
						use_upper = true;
					} else if (lower_has_intersection && !upper_has_intersection) {
						use_upper = false;
					} else {
						auto I0 = ea.End(lower_intersection->GetPiece(CI));
						auto I1 = ea.End(upper_intersection->GetPiece(CI));

						if (Positive(ea.CalcateVertexRelativePosition(I0, I1))) {
							use_upper = false;
						} else {
							use_upper = true;
						}
					}
				}

				VEventGroup group{};
				if (use_upper) {
					// 如果 AB 分为了 AI 和 IB, 则裂解 AB
					if (upper_intersection->HasPiece(IB)) {
						auto spr = SplitEvent(it->e,
							upper_intersection->GetPiece(AI),
							upper_intersection->GetPiece(IB)
						);
						// 将 it 指向的起始点事件替换为 AI 的起始点事件
						*it = MakeStartVertexEvent(spr.aiEvent.id);

						// 重新加入事件: AI 的终点事件、IB 的起点事件
						// 无视 IB 的终点事件
						group.Add(
							spr.aiEvent.id, false,
							spr.ibEvent.id, true
						);

						// 重新加入事件: AI 的终点事件、IB 的起点事件、IB 的终点事件
						//veq.Push(MakeEndVertexEvent(spr.aiEvent.id));
						//veq.Push(MakeStartVertexEvent(spr.ibEvent.id));
						//veq.Push(MakeEndVertexEvent(spr.ibEvent.id));
					}

					if (upper_intersection->HasPiece(ID)) {
						auto spr = SplitEvent(cur.e,
							upper_intersection->GetPiece(CI),
							upper_intersection->GetPiece(ID)
						);

						CalcGroupWind(events.begin(), it) >> spr.aiEvent;

						// 在 it 前插入 CI 的起点事件
						auto new_loc = events.emplace(it, MakeStartVertexEvent(spr.aiEvent.id));
						events_begin = new_loc;
						events_end = events_begin + 1;

						if (events.cbegin() != new_loc) {
							// 如果新加入的活跃边下方有别的活跃边, 则需要记录下方第一条边
							GetEdgeEvent(new_loc->e).firstBottom = (new_loc - 1)->e;
						}

						// 重新加入事件: CI 的终点事件、ID 的起点事件
						// 无视 ID 的终点事件
						group.Add(
							spr.aiEvent.id, false,
							spr.ibEvent.id, true
						);

						// 重新加入事件: CI 的终点事件、ID 的起点事件、ID 的终点事件
						//veq.Push(MakeEndVertexEvent(spr.aiEvent.id));
						//veq.Push(MakeStartVertexEvent(spr.ibEvent.id));
						//veq.Push(MakeEndVertexEvent(spr.ibEvent.id));
					} else {
						CalcGroupWind(events.begin(), it) >> GetEdgeEvent(cur.e);

						// 在 it 前插入 CD 的起点事件
						auto new_loc = events.emplace(it, cur);
						events_begin = new_loc;
						events_end = events_begin + 1;

						if (events.cbegin() != new_loc) {
							// 如果新加入的活跃边下方有别的活跃边, 则需要记录下方第一条边
							GetEdgeEvent(new_loc->e).firstBottom = (new_loc - 1)->e;
						}
					}
				} else {
					// 如果 AB 分为了 AI 和 IB, 则裂解 AB
					if (lower_intersection->HasPiece(IB)) {
						auto spr = SplitEvent((it - 1)->e,
							lower_intersection->GetPiece(AI),
							lower_intersection->GetPiece(IB)
						);
						// 将 it-1 指向的起始点事件替换为 AI 的起始点事件
						*(it - 1) = MakeStartVertexEvent(spr.aiEvent.id);

						group.Add(
							spr.aiEvent.id, false,
							spr.ibEvent.id, true
						);

						// 重新加入事件: AI 的终点事件、IB 的起点事件、IB 的终点事件
						//veq.Push(MakeEndVertexEvent(spr.aiEvent.id));
						//veq.Push(MakeStartVertexEvent(spr.ibEvent.id));
						//veq.Push(MakeEndVertexEvent(spr.ibEvent.id));
					}

					if (lower_intersection->HasPiece(ID)) {
						auto spr = SplitEvent(cur.e,
							lower_intersection->GetPiece(CI),
							lower_intersection->GetPiece(ID)
						);

						CalcGroupWind(events.begin(), it) >> spr.aiEvent;

						// 在 it 前插入 CI 的起点事件
						auto new_loc = events.emplace(it, MakeStartVertexEvent(spr.aiEvent.id));
						events_begin = new_loc;
						events_end = events_begin + 1;

						if (events.cbegin() != new_loc) {
							// 如果新加入的活跃边下方有别的活跃边, 则需要记录下方第一条边
							GetEdgeEvent(new_loc->e).firstBottom = (new_loc - 1)->e;
						}

						group.Add(
							spr.aiEvent.id, false,
							spr.ibEvent.id, true
						);

						// 重新加入事件: CI 的终点事件、ID 的起点事件、ID 的终点事件
						//veq.Push(MakeEndVertexEvent(spr.aiEvent.id));
						//veq.Push(MakeStartVertexEvent(spr.ibEvent.id));
						//veq.Push(MakeEndVertexEvent(spr.ibEvent.id));
					} else {
						CalcGroupWind(events.begin(), it) >> GetEdgeEvent(cur.e);

						// 在 it 前插入 CD 的起点事件
						auto new_loc = events.emplace(it, cur);
						events_begin = new_loc;
						events_end = events_begin + 1;

						if (events.cbegin() != new_loc) {
							// 如果新加入的活跃边下方有别的活跃边, 则需要记录下方第一条边
							GetEdgeEvent(new_loc->e).firstBottom = (new_loc - 1)->e;
						}
					}
				}
				if (group.edges[0] != npos) {
					veq.Push(group);
				}
			}
		}

		void AcceptSameLocEndEvents(std::vector<VEvent>& end_events, TopoVertex& vertex);

		inline const EdgeEvent& GetEdgeEvent(Handle handle) const {
			return edgeEvents[handle];
		}
		inline EdgeEvent& GetEdgeEvent(Handle handle) {
			return edgeEvents[handle];
		}
		inline const TopoVertex& GetVertexEvent(Handle handle) const {
			return topoVertices[handle];
		}
		inline TopoVertex& GetVertexEvent(Handle handle) {
			return topoVertices[handle];
		}

		template<class E>
		EdgeEvent MakeEdgeEvent(E&& e, Handle id, bool is_clipper) {
			EdgeEvent ee{ std::forward<E>(e) };
			ee.id = id;
			ee.isClipper = is_clipper;
			return ee;
		}

		VertexEvent MakeStartVertexEvent(Handle e) {
			return VertexEvent{
				/*ea.Start(edgeEvents[e].edge),*/ e, true
			};
		}

		VertexEvent MakeEndVertexEvent(Handle e) {
			return VertexEvent{
				/*ea.End(edgeEvents[e].edge),*/ e, false
			};
		}

		// 注册边, 返回边事件(注册新边后, 数组可能扩容, 导致就旧引用失效)
		template<class E>
		inline EdgeEvent& RegisterEdge(E&& edge) {
			Handle edge_handle = static_cast<Handle>(edgeEvents.size());
			return edgeEvents.emplace_back(std::forward<E>(edge), edge_handle);
		}

		template<class... E>
		auto RegisterEdge(E&&... edge) {
			size_t start = edgeEvents.size();

			// 逐个注册
			(RegisterEdge(std::forward<E>(edge)), ...);

			// 一次性返回所有注册的边事件, 防止扩容导致引用失效
			return GetEdgeEvents(start, std::make_index_sequence<sizeof...(E)>());
		}

		auto& RegisterVertex() {
			Handle id = static_cast<Handle>(topoVertices.size());
			auto& res = topoVertices.emplace_back();
			res.id = id;
			return res;
		}

		template<size_t... Index>
		auto GetEdgeEvents(size_t start, std::index_sequence<Index...>) {
			return std::tie(edgeEvents[start + Index]...);
		}

		decltype(auto) GetPointInVertexEvent(const VEvent& ve) {
			assert(!GetEdgeEvent(ve.e).discarded);
			return ve.start ? ea.Start(GetEdgeEvent(ve.e).edge) : ea.End(GetEdgeEvent(ve.e).edge);
		}

		// 组
		struct CopyGroup {
			inline void operator()(EdgeEvent& derived, const EdgeEvent& base) const {
				derived.isClipper = base.isClipper;
			}
		};

		// 下方第一条边
		struct CopyFirstBottomEdge {
			inline void operator()(EdgeEvent& derived, const EdgeEvent& base) const {
				derived.firstBottom = base.firstBottom;
			}
		};

		// 追踪分割后的第一条边
		struct TrackFirstSegmentedEdge {
			inline void operator()(const EdgeEvent& derived, EdgeEvent& base) const {
				base.firstSplit = derived.id;
			}
		};

		struct MergeFirstSegmentedEdge {
			inline void operator()(const EdgeEvent& derived, EdgeEvent& base) const {
				base.firstMerge = derived.id;
			}
		};
		struct TrackSecondSegmentedEdge {
			inline void operator()(const EdgeEvent& derived, EdgeEvent& base) const {
				base.secondSplit = derived.id;
			}
		};

		// 环绕数
		struct CopyWinds {
			inline void operator()(EdgeEvent& derived, const EdgeEvent& base) const {
				derived.clipperWind = base.clipperWind;
				derived.subjectWind = base.subjectWind;
			}
		};

		// 反转性
		struct CopyPolarity {
			inline void operator()(EdgeEvent& derived, const EdgeEvent& base) const {
				derived.reversed = base.reversed;
			}
		};

		// 反转的反转性
		struct InvertPolarity {
			inline void operator()(EdgeEvent& derived, const EdgeEvent& base) const {
				derived.reversed = !base.reversed;
			}
		};

		// 单调性
		struct CopyMonotonicity {
			inline void operator()(EdgeEvent& derived, const EdgeEvent& base) const {
				derived.monotonicity = base.monotonicity;
			}
		};

		// 反转的单调性
		struct InvertMonotonicity {
			inline void operator()(EdgeEvent& derived, const EdgeEvent& base) const {
				derived.monotonicity = static_cast<VertexRelativePositionType>(
					-static_cast<int>(base.monotonicity)
					);
			}
		};

		// 源id
		struct CopySource {
			inline void operator()(EdgeEvent& derived, const EdgeEvent& base) const {
				derived.source = base.id;
			}
		};

		// 起点
		struct CopyStartingVertex {
			inline void operator()(EdgeEvent& derived, const EdgeEvent& base) const {
				derived.startPntGroup = base.startPntGroup;
			}
		};

		// 终点
		struct CopyEndVertex {
			inline void operator()(EdgeEvent& derived, const EdgeEvent& base) const {
				derived.endPntGroup = base.endPntGroup;
			}
		};

		// 抛弃旧事件
		struct DiscardBase {
			inline void operator()(const EdgeEvent& derived, EdgeEvent& base) const {
				base.discarded = true;
			}
		};

		// 聚合边id
		struct CopyAggregatedEdges {
			inline void operator()(EdgeEvent& derived, EdgeEvent& base) const {
				if (base.aggregatedEdges) {
					derived.aggregatedEdges = std::make_unique<AggregatedEdgeEvent>(*base.aggregatedEdges);
				}
			}
		};

		// 合并聚合边
		struct MergeAggregatedEdges {
			void operator()(EdgeEvent& derived, EdgeEvent& base) const {
				if (!derived.IsAggregatedEdge()) {
					derived.aggregatedEdges = std::make_unique<AggregatedEdgeEvent>();
					derived.aggregatedEdges->sourceEdges.reserve(2);
					// 自身id本可以不保存, 但为了编程方便, 还是保存
					derived.aggregatedEdges->sourceEdges.emplace_back(derived.id);
				}

				auto& das = derived.aggregatedEdges->sourceEdges;
				if (base.IsAggregatedEdge()) {
					auto& bas = base.aggregatedEdges->sourceEdges;
					das.insert(das.end(), bas.begin(), bas.end());
				} else {
					das.emplace_back(base.id);
				}
			}
		};

		template<class... Inherited>
		inline void Inherit(EdgeEvent& derived, EdgeEvent& base) {
			(Inherited{}(derived, base), ...);
		}

		struct HandleCoincidentEdgesResult {
			Handle coincidentEdges = npos;
			Handle ib = npos;
			Handle id = npos;
		};

		template<class EdgeRelativePositionResult>
		HandleCoincidentEdgesResult HandleCoincidentEdges(Handle ab_handle, Handle cd_handle, EdgeRelativePositionResult&& result) {
			assert(result.HasPiece(CI));
			assert(CurveRelativePositionType::Coincident == result.RelativePositionType());

			// 该函数的 TrackFirstSegmentedEdge 和 TrackSecondSegmentedEdge 极有可能存在bug
			// 需要测试

			//TrackFirstSegmentedEdge 在算法(仅针对聚合边)中已无意义, 当前事件点不会再加入集合

			if (result.HasPiece(IB) && result.HasPiece(ID)) {
				// AI 和 CI 重合,IB 和 ID 不重合
				// 注册AI, IB, 和 ID, 但 CI 不会注册
				auto [ai_event, ib_event, id_event] = RegisterEdge(
					result.GetPiece(AI), result.GetPiece(IB), result.GetPiece(ID)
				);

				auto& ab_event = GetEdgeEvent(ab_handle);
				auto& cd_event = GetEdgeEvent(cd_handle);

				// IB 继承 AB 属性
				Inherit<
					DiscardBase, CopySource, CopyGroup,
					CopyPolarity, CopyMonotonicity, CopyAggregatedEdges,
					TrackSecondSegmentedEdge
				>(ib_event, ab_event);

				// ID 继承 CD 属性
				Inherit<
					DiscardBase, CopySource, CopyGroup,
					CopyPolarity, CopyMonotonicity, CopyAggregatedEdges,
					TrackSecondSegmentedEdge
				>(id_event, cd_event);

				// AI 继承 AB 属性
				Inherit<
					DiscardBase, CopySource, CopyGroup, CopyPolarity,
					CopyMonotonicity, CopyAggregatedEdges,
					TrackFirstSegmentedEdge/*, CopyStartingVertex*/
				>(ai_event, ab_event);

				// 将 CI 合并到 AI 中
				Inherit<MergeAggregatedEdges, MergeFirstSegmentedEdge>(ai_event, cd_event);

				return HandleCoincidentEdgesResult{ ai_event.id, ib_event.id, id_event.id };
			} else if (!result.HasPiece(IB) && result.HasPiece(ID)) {
				// AB 和 CI 重合, 余留下 ID

				// 仅注册 ID
				auto& id_event = RegisterEdge(result.GetPiece(ID));
				auto& ab_event = GetEdgeEvent(ab_handle);
				auto& cd_event = GetEdgeEvent(cd_handle);

				// ID 继承 CD 属性
				Inherit<
					DiscardBase, CopySource, CopyGroup,
					CopyPolarity, CopyMonotonicity, CopyAggregatedEdges,
					TrackSecondSegmentedEdge
				>(id_event, cd_event);

				// 将 CI 合并到 AB 中
				Inherit<MergeAggregatedEdges, MergeFirstSegmentedEdge>(ab_event, cd_event);

				return HandleCoincidentEdgesResult{ ab_event.id, npos, id_event.id };
			} else if (result.HasPiece(IB) && !result.HasPiece(ID)) {
				// CD 和 AI 重合, 余留下 IB

				// 仅注册 IB
				auto& ib_event = RegisterEdge(result.GetPiece(IB));
				auto& ab_event = GetEdgeEvent(ab_handle);
				auto& cd_event = GetEdgeEvent(cd_handle);

				// IB 继承 AB 属性
				Inherit<
					DiscardBase, CopySource, CopyGroup,
					CopyPolarity, CopyMonotonicity, CopyAggregatedEdges,
					TrackSecondSegmentedEdge
				>(ib_event, ab_event);

				// 将 AI 合并到 CD 中
				Inherit<MergeAggregatedEdges, MergeFirstSegmentedEdge>(cd_event, ab_event);

				return HandleCoincidentEdgesResult{ cd_event.id, ib_event.id, npos };
			} else {
				auto& ab_event = GetEdgeEvent(ab_handle);
				auto& cd_event = GetEdgeEvent(cd_handle);

				// AB 和 CD 完全重合
				Inherit<
					DiscardBase, MergeAggregatedEdges,
					MergeFirstSegmentedEdge
				>(ab_event, cd_event);

				return HandleCoincidentEdgesResult{ ab_event.id, npos, npos };
			}
		}

		template<class OutPutContainer>
		void RemoveDiscardedEvent(OutPutContainer& events) {
			events.erase(std::remove_if(events.begin(), events.end(),
				[this](const VEvent& ve) {
					assert(!edgeEvents[ve.e].discarded);
					return edgeEvents[ve.e].discarded;
				}), events.end());
		}

		struct GroupWindResult {
			Int clipperWind = 0;
			Int subjectWind = 0;

			void operator>>(EdgeEvent& ee) const {
				ee.clipperWind = clipperWind;
				ee.subjectWind = subjectWind;
			}
		};

		/**
		 * @brief 计算环绕数, TODO 优化性能: 可以仅通过 firstBottom 来计算
		 * @tparam EventsIterator
		 * @param begin
		 * @param end
		 * @return
		 */
		template<class EventsIterator>
		GroupWindResult CalcGroupWind(EventsIterator begin, EventsIterator end) const {
			GroupWindResult winds{};
			if (begin == end) return winds;
			const auto& ee = GetEdgeEvent((end - 1)->e);
			winds.clipperWind = ee.clipperWind;
			winds.subjectWind = ee.subjectWind;
			if (ee.aggregatedEdges) TAILOR_UNLIKELY{
				for (auto ae : ee.aggregatedEdges->sourceEdges) {
					auto& aee = GetEdgeEvent(ae);
					auto& wind = aee.isClipper ? winds.clipperWind : winds.subjectWind;
					wind += aee.reversed ? -1 : +1;
				}
			} else {
				auto& wind = ee.isClipper ? winds.clipperWind : winds.subjectWind;
				wind += ee.reversed ? -1 : +1;
			}
			return winds;

			//GroupWindResult winds{};
			//for (auto it = begin; it != end; ++it) {
			//	const auto& ee = GetEdgeEvent(it->e);
			//	assert(!ee.discarded);
			//	if (ee.aggregatedEdges) {
			//		for (auto ae : ee.aggregatedEdges->sourceEdges) {
			//			auto& aee = GetEdgeEvent(ae);
			//			auto& wind = aee.isClipper ? winds.clipperWind : winds.subjectWind;
			//			wind += aee.reversed ? -1 : +1;
			//		}
			//	} else {
			//		auto& wind = ee.isClipper ? winds.clipperWind : winds.subjectWind;
			//		wind += ee.reversed ? -1 : +1;
			//	}
			//}
			//return winds;
		}

		void CreateEvents(const std::vector<Edge>& clipper, const std::vector<Edge>& subject) {
			for (const auto& e : clipper) {
				RegisterEdge(e).isClipper = true;
			}
			for (const auto& e : subject) {
				RegisterEdge(e).isClipper = false;
			}

			// 单调性分割
			//for (size_t i = 0, n = edgeEvents.size(); i < n; ++i) {
			//	MonotonicSplitResult<Edge> split_res = ea.SplitToMonotonic(GetEdgeEvent(i).edge);
			//	if (!split_res.remaining.has_value()) continue;
			//	bool keep_on = false;
			//	do {
			//		// 注册split_res.monotonous;
			//		auto& new_event = RegisterEdge(std::move(split_res.monotonous));
			//		Inherit<DiscardBase, CopySource, CopyGroup>(new_event, GetEdgeEvent(i));
			//		if (keep_on = split_res.remaining.has_value()) {
			//			split_res = ea.SplitToMonotonic(std::move(*split_res.remaining));
			//		}
			//	} while (keep_on);
			//}

			for (size_t i = 0, n = edgeEvents.size(); i < n; ++i) {
				auto& old_event = GetEdgeEvent(i);
				if (old_event.discarded) continue;

				auto& edge = edgeEvents[i].edge;
				old_event.monotonicity = ea.CalcateVertexRelativePosition(ea.Start(edge), ea.End(edge));

				if (!Negative(old_event.monotonicity)) continue;

				// old_event 可能失效, 禁止使用
				auto& new_event = RegisterEdge(ea.Reverse(edge));

				Inherit<
					DiscardBase, CopySource, CopyGroup,
					InvertPolarity, InvertMonotonicity
				>(new_event, GetEdgeEvent(i));
			}
		}

		struct SplitEventResult {
			EdgeEvent& aiEvent;
			EdgeEvent& ibEvent;
		};

		template<class E0, class E1>
		auto SplitEvent(Handle ab_handle, E0&& ai, E1&& ib) {
			auto [ai_event, ib_event] = RegisterEdge(
				std::forward<E0>(ai), std::forward<E1>(ib)
			);
			EdgeEvent& ab_event = GetEdgeEvent(ab_handle);

			// AI 需要额外复制起点信息
			Inherit<
				DiscardBase, CopySource, CopyGroup, CopyWinds,
				CopyPolarity, CopyMonotonicity, CopyAggregatedEdges,
				CopyStartingVertex, CopyEndVertex, CopyFirstBottomEdge,
				TrackFirstSegmentedEdge
			>(ai_event, ab_event);
			Inherit<
				DiscardBase, CopySource, CopyGroup, CopyWinds,
				CopyPolarity, CopyMonotonicity, CopyAggregatedEdges,
				TrackSecondSegmentedEdge
			>(ib_event, ab_event);
			return SplitEventResult{ ai_event, ib_event };
		}

		void EndEvent(EdgeEvent& e) {
			e.end = true;

			// 该边的终点指向该边
			TopoVertex& end_vertex = GetVertexEvent(e.endPntGroup);
			end_vertex.endGroup.Insert(e.id);

			// 该边的起点指向该边
			TopoVertex& start_vertex = GetVertexEvent(e.startPntGroup);
			bool success = start_vertex.startGroup.ReplaceEvent(e.id, this->edgeEvents);
			assert(success);
		}

		void InsertVertexEvent(std::vector<VEvent>& start_events, const VEvent& new_event) {
			if (GetEdgeEvent(new_event.e).discarded) return;

			if (start_events.empty()) {
				start_events.emplace_back(new_event);
				return;
			}

			// 二分插入
			// 不过大部分情况下其实没什么用, 同一个点包含多条事件边还是少数
			auto begin = start_events.begin();
			auto end = start_events.end();

			while (begin < end) {
				// middle
				auto it = begin + std::distance(begin, end) / 2;

				auto rp = ea.CalcateEdgeRelativePosition(
					GetEdgeEvent(it->e).edge, GetEdgeEvent(new_event.e).edge,
					OnlyFocusOnRelativePositionAndCoincidenceDetails{}
				);

				//assert(rp.HasPiece(CI)); // 理论上此处不可能出现无 CI 的情况, 但是有可能出现重合的情况

				// 处理重合边
				if (CurveRelativePositionType::Coincident == rp.RelativePositionType()) {
					auto info = HandleCoincidentEdges(it->e, new_event.e, rp);

					auto& old_edge = GetEdgeEvent(it->e);

					VEventGroup group{};

					if (old_edge.discarded) {
						// 如果旧边被废弃, 则替换, 并加入对应终点事件
						*it = MakeStartVertexEvent(info.coincidentEdges);
					}

					if (npos != info.ib) {
						// 存在 IB
						group.Add(info.ib, true);
					}

					if (npos != info.id) {
						// 存在 ID
						group.Add(info.id, true);
					}

					if (group.edges[0] != npos) {
						veq.Push(group);
					}
					return;
				}

				if (rp.RelativePositionType() == CurveRelativePositionType::Upward) {
					begin = it + 1;
				} else {
					end = it;
				}
			}

			start_events.emplace(begin, new_event);
		}

		size_t CalcVaildEdgeSize(const std::vector<VEvent>& es) const {
			return std::accumulate(es.begin(), es.end(), 0, [this](size_t count, const VEvent& e) {
				const auto& edge = GetEdgeEvent(e.e);
				return count + (edge.aggregatedEdges ?
					edge.aggregatedEdges->sourceEdges.size() : 1);
				});
		}
	public:
		std::vector<VEvent> events; // 活跃的边事件集合，其相邻的两个事件边不允许出现相交的情况, 规定从下到上排列
		EdgeAnalyzer& ea;			// 边分析器
		VEQ& veq;
		std::vector<VEvent> cache;

		// TODO: vector 的扩容机制实在太恶心了, 要不改成 dequee ? 但对事件的频繁搜索是否会显著降低效率?
		std::vector<EdgeEvent>& edgeEvents; // 事件边集合
		//std::vector<AggregatedEdgeEvent> aggregatedEdgeEvents; // 聚合边事件, 弃用, 改为存储在 EdgeEvent 内
		std::vector<TopoVertex> topoVertices; // 拓扑顶点集合
	};

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
		using EdgeEvent = Tailor::EdgeEvent;

		std::vector<EdgeEvent> edgeEvent;
		std::vector<TopoVertex> vertexEvents;
	};

	PatternDrafting Execute() {
		std::vector<VEventGroup> container;
		std::vector<EdgeEvent> edge_events;

		//container.reserve(20000);
		container.reserve(static_cast<size_t>((clipper.size() + subject.size()) * 2.5));
		VertexEventQueue<> queue(ea, edge_events, std::move(container));
		EdgeStateSet<decltype(queue)> set(queue, edge_events, ea, clipper, subject);

		// 顶点事件入队

		for (size_t i = 0, n = set.edgeEvents.size(); i < n; ++i) {
			auto& edge = set.edgeEvents[i];
			if (edge.discarded) continue;
			queue.Push(i, true);
			queue.Push(i, false);
			//queue.Push(ea.Start(edge.edge), i, true);
			//queue.Push(ea.End(edge.edge), i, false);
		}

		std::vector<VEvent> start_events;
		std::vector<VEvent> end_events;

		//size_t times = 0;
		// 顶点事件出队
		while (!queue.Empty()) {
			//times++;

			//if (times == 130) {
			//	int ccc = 0;
			//}

			// 取出所有的相同位置的顶点事件
			queue.PopAllEvents(start_events, end_events);
			// 现在应该可以禁用 RemoveDiscardedEvent 了
			//set.RemoveDiscardedEvent(start_events);
			//set.RemoveDiscardedEvent(end_events);

			// 事件处理的有效边总数必须为偶数
			assert((
				set.CalcVaildEdgeSize(start_events) +
				set.CalcVaildEdgeSize(end_events))
				% 2 == 0);

			//for (auto& se : start_events)
			//{
			//	const auto& edge = set.GetEdgeEvent(se.e);
			//	this->DebugFunc3(edge, se.start);
			//}

			//for (auto& ee : end_events)
			//{
			//	const auto& edge = set.GetEdgeEvent(ee.e);
			//	this->DebugFunc3(edge, ee.start);
			//}

			auto& vertex = set.RegisterVertex();

			set.PreprocessEvents(start_events, end_events);

			set.AcceptSameLocEndEvents(end_events, vertex);

			set.AcceptSameLocStartEvents(start_events, vertex);

			end_events.clear();
			start_events.clear();
		}
		return PatternDrafting{ std::move(edge_events),std::move(set.topoVertices) };
	}

private:

private:
};

template<EdgeConcept Edge, EdgeAnalyzerConcept<Edge> EdgeAnalyzer>
template<class VEQ>
inline void Tailor<Edge, EdgeAnalyzer>::EdgeStateSet<VEQ>::
AcceptSameLocEndEvents(std::vector<VEvent>& end_events, TopoVertex& vertex) {
	for (auto begin = events.begin(); begin != events.end(); ++begin) {
		if (end_events.empty()) break;

		auto end = begin; // 需要移除的区间末尾
		auto event_end = events.end();
		for (; !end_events.empty() && end != events.end(); ++end) {
			auto remove_it = std::remove_if(end_events.begin(), end_events.end(),
				[eid = end->e](const VEvent& ve) {return ve.e == eid; }
			);
			if (end_events.end() == remove_it) break;
			end_events.erase(remove_it, end_events.end());
		}

		// 没有需要要删除的事件
		if (begin == end) continue;

		// 结束连续的事件
		std::for_each(begin, end, [this, &vertex](const VEvent& ve) {
			EdgeEvent& ee = GetEdgeEvent(ve.e);
			ee.endPntGroup = vertex.id;
			EndEvent(ee); // 结束被移除的事件
			});

		// 删除 [begin, end)
		begin = events.erase(begin, end);

		if (events.end() == begin /* 上方无边 */) break;
		if (events.begin() == begin /* 下方无边 */) continue;

		//VertexRelativePositionType vrp = ea.CalcateVertexRelativePosition((begin - 1)->v, begin->v);
		const VertexRelativePositionType vrp = ea.CalcateVertexRelativePosition(
			GetPointInVertexEvent(*(begin - 1)), GetPointInVertexEvent(*begin)
		);

		const bool reversed = Negative(vrp);

		// 由于比较的两个事件可能不满足 A <= C, 因此需要判断
		const auto ab_it = reversed ? begin : (begin - 1);
		const auto cd_it = reversed ? (begin - 1) : begin;

		// 移除事件后, 将上下的边事件进行求交处理
		auto erp = ea.CalcateEdgeRelativePosition(
			GetEdgeEvent(ab_it->e).edge,
			GetEdgeEvent(cd_it->e).edge,
			OnlyFocusOnRelativePositionDetailsWithoutCoincidence{}
		);

		//const auto& debug_edge0 = GetEdgeEvent(ab_it->e);
		//const auto& debug_edge1 = GetEdgeEvent(cd_it->e);
		assert(erp.HasPiece(CI));

		VEventGroup group{};
		if (erp.HasPiece(IB)) {
			auto spr = SplitEvent(ab_it->e, erp.GetPiece(AI), erp.GetPiece(IB));

			// 将 ab_it 指向的起始点事件替换为 AI 的起始点事件
			*ab_it = MakeStartVertexEvent(spr.aiEvent.id);

			group.Add(
				spr.aiEvent.id, false,
				spr.ibEvent.id, true
			);

			// 重新加入事件: AI 的终点事件、IB 的起点事件、IB 的终点事件
			//veq.Push(MakeEndVertexEvent(spr.aiEvent.id));
			//veq.Push(MakeStartVertexEvent(spr.ibEvent.id));
			//veq.Push(MakeEndVertexEvent(spr.ibEvent.id));
		}

		if (erp.HasPiece(ID)) {
			auto spr = SplitEvent(cd_it->e, erp.GetPiece(CI), erp.GetPiece(ID));

			// 将 cd_it 指向的起始点事件替换为 CI 的起始点事件
			*cd_it = MakeStartVertexEvent(spr.aiEvent.id);

			group.Add(
				spr.aiEvent.id, false,
				spr.ibEvent.id, true
			);

			// 重新加入事件: CI 的终点事件、ID 的起点事件、ID 的终点事件
			//veq.Push(MakeEndVertexEvent(spr.aiEvent.id));
			//veq.Push(MakeStartVertexEvent(spr.ibEvent.id));
			//veq.Push(MakeEndVertexEvent(spr.ibEvent.id));
		}
		if (group.edges[0] != npos) {
			veq.Push(group);
		}
	}
}

TAILOR_NAMESPACE_END