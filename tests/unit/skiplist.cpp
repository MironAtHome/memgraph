#include <iostream>

#include "data_structures/skiplist/skiplist.hpp"
#include "utils/assert.hpp"

using std::cout;
using std::endl;

using skiplist_t = SkipList<int, int>;

void print_skiplist(const skiplist_t::Accessor &skiplist)
{
    cout << "---- skiplist now has: ";

    for (auto &kv : skiplist)
        cout << "(" << kv.first << ", " << kv.second << ") ";

    cout << "----" << endl;
}

int main(void)
{
    skiplist_t skiplist;
    auto accessor = skiplist.access();

    // insert 10
    permanent_assert(accessor.insert_unique(1, 10).second == true,
                     "add first element");

    // try insert 10 again (should fail)
    permanent_assert(accessor.insert_unique(1, 10).second == false,
                     "add the same element, should fail");

    // insert 20
    permanent_assert(accessor.insert_unique(2, 20).second == true,
                     "insert new unique element");

    print_skiplist(accessor);

    // value at key 3 shouldn't exist
    permanent_assert((accessor.find(3) == accessor.end()) == true,
                     "try to find element which doesn't exist");

    // value at key 2 should exist
    permanent_assert((accessor.find(2) != accessor.end()) == true,
                     "find iterator");

    // at key 2 is 20 (true)
    permanent_assert(accessor.find(2)->second == 20, "find element");

    // removed existing (1)
    permanent_assert(accessor.remove(1) == true, "try to remove element");

    // removed non-existing (3)
    permanent_assert(accessor.remove(3) == false,
                     "try to remove element which doesn't exist");

    // insert (1, 10)
    permanent_assert(accessor.insert_unique(1, 10).second == true,
                     "insert unique element");

    // insert (4, 40)
    permanent_assert(accessor.insert_unique(4, 40).second == true,
                     "insert unique element");

    print_skiplist(accessor);

    return 0;
}
