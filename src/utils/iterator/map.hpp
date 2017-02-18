#pragma once

#include "utils/iterator/composable.hpp"
#include "utils/iterator/iterator_base.hpp"

namespace iter {

// Class which maps values returned by I iterator into value of type T with OP
// function.
// T - type of return value
// I - iterator type
// OP - type of mapper function
template <class T, class I, class OP>
class Map : public IteratorBase<T>, public Composable<T, Map<T, I, OP>> {
 public:
  Map() = delete;

  // Map operation is designed to be used in chained calls which operate on a
  // iterator. Map will in that usecase receive other iterator by value and
  // std::move is a optimization for it.
  Map(I &&iter, OP &&op) : iter(std::move(iter)), op(std::move(op)) {}

  Map(Map &&m) : iter(std::move(m.iter)), op(std::move(m.op)) {}

  ~Map() final {}

  Option<T> next() final {
    auto item = iter.next();
    if (item.is_present()) {
      return Option<T>(op(item.take()));
    } else {
      return Option<T>();
    }
  }

  Count count() final { return iter.count(); }

 private:
  I iter;
  OP op;
};

template <class I, class OP>
auto make_map(I &&iter, OP &&op) {
  // Compiler cant deduce type T. decltype is here to help with it.
  return Map<decltype(op(iter.next().take())), I, OP>(std::move(iter),
                                                      std::move(op));
}
}
