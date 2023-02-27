#ifndef KOUEK_STATEFUL_H
#define KOUEK_STATEFUL_H

#include <cassert>
#include <functional>

#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace kouek {
class StatefulSystem {
  private:
    std::vector<bool> modifieds;
    std::unordered_map<const void *, size_t> addrToIndices;

    struct FuncNode {
        size_t sortedSrcNum = 0, sortedDstNum = 0, topNum = 0;
        std::function<void()> func;
        std::function<bool()> pred;
        std::vector<size_t> srcs;
        std::vector<size_t> dsts;

        FuncNode(const std::function<void()> &func,
                 const std::function<bool()> &pred = nullptr)
            : func(func), pred(pred) {}
        FuncNode(const FuncNode &other)
            : func(other.func), pred(other.pred), srcs(other.srcs),
              dsts(other.dsts) {
            topNum = other.topNum;
            sortedSrcNum = other.sortedSrcNum;
            sortedDstNum = other.sortedDstNum;
        };
        FuncNode(FuncNode &&right) noexcept
            : func(std::move(right.func)), pred(std::move(right.pred)),
              srcs(std::move(right.srcs)), dsts(std::move(right.dsts)) {
            topNum = right.topNum;
            sortedSrcNum = right.sortedSrcNum;
            sortedDstNum = right.sortedDstNum;
        }
        FuncNode &operator=(const FuncNode &other) {
            func = other.func;
            pred = other.pred;
            srcs = other.srcs;
            dsts = other.dsts;
            topNum = other.topNum;
            sortedSrcNum = other.sortedSrcNum;
            sortedDstNum = other.sortedDstNum;
            return *this;
        }
        FuncNode &operator=(FuncNode &&right) noexcept {
            func = std::move(right.func);
            pred = std::move(right.pred);
            srcs = std::move(right.srcs);
            dsts = std::move(right.dsts);
            topNum = right.topNum;
            sortedSrcNum = right.sortedSrcNum;
            sortedDstNum = right.sortedDstNum;
            return *this;
        }
    };
    std::vector<FuncNode> unsortedFuncs;
    std::vector<FuncNode> sortedFuncs;
    std::vector<FuncNode> everyTimeExecFuncs;

  private:
    template <size_t Curr, typename TupTy>
    void forTupElemDo(const std::function<void(const void *)> &func,
                      const TupTy &tuple) {
        if constexpr (Curr < std::tuple_size_v<TupTy>) {
            static_assert(
                std::is_reference_v<std::tuple_element_t<Curr, TupTy>>,
                "Stateful variables should be passed by reference!");
            func((const void *)&std::get<Curr>(tuple));
            forTupElemDo<Curr + 1>(func, tuple);
        }
    }

    inline auto registerAddr(const void *addr) {
        auto varIdx = addrToIndices.size();
        if (addrToIndices.find(addr) == addrToIndices.end()) {
            addrToIndices.emplace(std::piecewise_construct,
                                  std::forward_as_tuple(addr),
                                  std::forward_as_tuple(varIdx));
            modifieds.emplace_back(false);
        } else
            varIdx = addrToIndices[addr];
        return varIdx;
    }

    void analyse() {
        // init dependency graph
        std::vector<size_t> inDegrees(addrToIndices.size(), 0);
        std::vector<std::vector<size_t>> outAdjs(addrToIndices.size());
        for (const auto &funcNode : unsortedFuncs)
            for (auto dstIdx : funcNode.dsts)
                for (auto srcIdx : funcNode.srcs) {
                    outAdjs[srcIdx].emplace_back(dstIdx);
                    ++inDegrees[dstIdx];
                }
        for (auto &outAdj : outAdjs) {
            // de-duplicate
            std::sort(outAdj.begin(), outAdj.end());
            auto dupStartItr = std::unique(outAdj.begin(), outAdj.end());
            for (auto itr = dupStartItr; itr != outAdj.end(); ++itr)
                --inDegrees[*itr];
            outAdj.erase(dupStartItr, outAdj.end());
        }

        // init function nodes
        while (!sortedFuncs.empty()) {
            unsortedFuncs.emplace_back(std::move(sortedFuncs.back()));
            sortedFuncs.pop_back();
        }
        for (auto &funcNode : unsortedFuncs) {
            funcNode.sortedSrcNum = funcNode.srcs.size();
            funcNode.sortedDstNum = funcNode.dsts.size();
            funcNode.topNum = 0;
        }

        std::queue<size_t> que;
        for (size_t idx = 0; idx < inDegrees.size(); ++idx)
            if (inDegrees[idx] == 0)
                que.emplace(idx);

        std::unordered_set<size_t> currSrcIndices;
        decltype(unsortedFuncs) unsortedBuf, sortedBuf;
        unsortedBuf.reserve(unsortedFuncs.size() / 2);
        sortedBuf.reserve(unsortedBuf.size());
        size_t topNum = 0;
        printf("Stateful System starts analysis...\n");
        while (!que.empty()) {
            // generate topology sorted list of variables
            currSrcIndices.clear();
            while (!que.empty()) {
                currSrcIndices.emplace(que.front());
                que.pop();
            }
            for (auto srcIdx : currSrcIndices)
                for (auto dstIdx : outAdjs[srcIdx]) {
                    --inDegrees[dstIdx];
                    if (inDegrees[dstIdx] == 0)
                        que.emplace(dstIdx);
                }
            ++topNum;
            printf("Stateful variables on topology layer %llu are: ", topNum);
            for (auto idx : currSrcIndices)
                printf("%llu ", idx);

            // check variables in current pass to sort functions
            while (!unsortedFuncs.empty()) {
                auto &funcNode = unsortedFuncs.back();
                for (auto dstIdx : funcNode.dsts)
                    if (currSrcIndices.find(dstIdx) != currSrcIndices.end())
                        --funcNode.sortedDstNum;
                if (funcNode.sortedSrcNum == 0 && funcNode.sortedDstNum == 0) {
                    sortedBuf.emplace_back(std::move(funcNode));
                    unsortedFuncs.pop_back();
                } else {
                    for (auto srcIdx : funcNode.srcs)
                        if (currSrcIndices.find(srcIdx) !=
                            currSrcIndices.end()) {
                            --funcNode.sortedSrcNum;
                            funcNode.topNum = topNum;
                        }
                    unsortedBuf.emplace_back(std::move(funcNode));
                    unsortedFuncs.pop_back();
                }
            }
            unsortedFuncs = std::move(unsortedBuf);
            std::sort(sortedBuf.begin(), sortedBuf.end(),
                      [&](const FuncNode &a, const FuncNode &b) {
                          if (a.topNum > b.topNum)
                              return true;
                          return false;
                      });
            while (!sortedBuf.empty()) {
                sortedFuncs.emplace_back(std::move(sortedBuf.back()));
                sortedBuf.pop_back();
            }

            printf("and %llu functions are left\n", unsortedFuncs.size());
        }
        size_t noDstFuncNum = 0;
        for (auto &funcNode : unsortedFuncs)
            if (funcNode.sortedSrcNum == 0 && funcNode.sortedDstNum == 0) {
                sortedFuncs.emplace_back(std::move(funcNode));
                unsortedFuncs.pop_back();
                ++noDstFuncNum;
            }
        printf("Analyse %llu functions without dst variables\n", noDstFuncNum);

        printf("...Stateful System ends analysis\n");
        assert(unsortedFuncs.empty());
    }

  public:
    template <typename... Src, typename... Dst>
    void Register(std::tuple<Src...> src, const std::function<void()> &func,
                  std::tuple<Dst...> dst) {
        unsortedFuncs.emplace_back(func);
        auto &funcNode = unsortedFuncs.back();

        forTupElemDo<0>(
            [&](const void *addr) {
                funcNode.srcs.emplace_back(registerAddr(addr));
            },
            src);
        forTupElemDo<0>(
            [&](const void *addr) {
                funcNode.dsts.emplace_back(registerAddr(addr));
            },
            dst);
    }

    template <typename... Src>
    void Register(std::tuple<Src...> src, const std::function<void()> &func) {
        unsortedFuncs.emplace_back(func);
        auto &funcNode = unsortedFuncs.back();

        forTupElemDo<0>(
            [&](const void *addr) {
                funcNode.srcs.emplace_back(registerAddr(addr));
            },
            src);
    }

    template <typename... Dst>
    void Register(const std::function<void()> &func, std::tuple<Dst...> dst,
                  const std::function<bool()> pred = nullptr) {
        unsortedFuncs.emplace_back(func, pred);
        auto &funcNode = unsortedFuncs.back();

        forTupElemDo<0>(
            [&](const void *addr) {
                funcNode.dsts.emplace_back(registerAddr(addr));
            },
            dst);

        everyTimeExecFuncs.emplace_back(funcNode);
    }

    template <typename... Dst>
    void RegisterExecOnce(const std::function<void()> &func,
                          std::tuple<Dst...> dst) {
        forTupElemDo<0>([&](const void *addr) { registerAddr(addr); }, dst);
        func();
    }

    template <typename... Dst> void SetModified(std::tuple<Dst...> vars) {
        forTupElemDo<0>(
            [&](const void *addr) {
                auto varIdx = registerAddr(addr);
                modifieds[varIdx] = true;
            },
            vars);
    }

    void Update() {
        if (!unsortedFuncs.empty())
            analyse();

        for (const auto &funcNode : everyTimeExecFuncs) {
            if (funcNode.pred && !funcNode.pred())
                continue;
            funcNode.func();
            for (auto dstIdx : funcNode.dsts)
                modifieds[dstIdx] = true;
        }

        for (const auto &funcNode : sortedFuncs) {
            bool shouldExec = false;
            for (auto srcIdx : funcNode.srcs)
                if (modifieds[srcIdx]) {
                    shouldExec = true;
                    break;
                }
            if (funcNode.pred)
                shouldExec &= funcNode.pred();
            if (shouldExec) {
                funcNode.func();
                for (auto dstIdx : funcNode.dsts)
                    modifieds[dstIdx] = true;
            }
        }

        modifieds.assign(modifieds.size(), false);
    }
};
} // namespace kouek

#endif // !KOUEK_STATEFUL_H
