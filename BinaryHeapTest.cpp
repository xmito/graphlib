#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include "BinaryHeap.h"
#include <iostream>
#include <list>

TEST_CASE( "BinHeap constructors" ) {
	SECTION("BinHeap(const Compare& = Compare())") {
		graphlib::BinHeap<int> bh_one;
		REQUIRE(bh_one.empty());
		REQUIRE(bh_one.size() == 0);
		graphlib::BinHeap<int, std::greater<int>> bh_two;
		REQUIRE(bh_two.empty());
		REQUIRE(bh_two.size() == 0);
	}
	SECTION("BinHeap(const Container&, const Compare& = Compare())") {
		std::vector<int> vec{1, 2, 3, 4, 5, 6, 7};
		graphlib::BinHeap<int> bh(vec);
		REQUIRE(bh.size() == 7);
		for (int i = 1; i < 8; ++i) {
			REQUIRE(bh.top() == i);
			bh.pop();
		}

		graphlib::BinHeap<int, std::greater<int>> bh_comp(vec);
		REQUIRE(bh_comp.size() == 7);
		for (int i = 7; i > 0; --i) {
			REQUIRE(bh_comp.top() == i);
			bh_comp.pop();
		}
	}
	SECTION("BinHeap(InputIt, InputIt, const Compare& = Compare())") {
		std::list<int> list{1, 2, 3, 4, 5, 6, 7, 8, 9};
		graphlib::BinHeap<int> bh(list.begin(), list.end(), std::less<int>());
		REQUIRE(bh.size() == 9);
		REQUIRE(bh.empty() == false);
		for (int i = 1; i < 10; ++i) {
			REQUIRE(bh.top() == i);
			bh.pop();
		}
	}
	SECTION("BinHeap(const BinHeap&)") {
		graphlib::BinHeap<int> bh;
		for (int i = 0; i < 40; ++i)
			bh.push(i);
		graphlib::BinHeap<int> bh_cp(bh);
		REQUIRE(bh_cp.size() == 40);
		REQUIRE(bh_cp.empty() == false);
		REQUIRE(bh_cp.top() == 0);
		REQUIRE(bh == bh_cp);
	}
	SECTION("BinHeap(BinHeap&&)") {
		graphlib::BinHeap<int> bh;
		for (int i = 0; i < 40; ++i)
			bh.push(i);
		graphlib::BinHeap<int> bh_mv(std::move(bh));
		REQUIRE(bh_mv.size() == 40);
		REQUIRE(bh_mv.empty() == false);
		REQUIRE(bh_mv.top() == 0);
		REQUIRE(bh.empty());
		REQUIRE(bh.size() == 0);
	}
}

TEST_CASE("push()") {
	graphlib::BinHeap<int> bh;
	SECTION("push(const T&)") {
		for (int i = 10; i > 0; --i) {
			graphlib::BinHeap<int>::Handle handle = bh.push(i);
			REQUIRE(bh.top() == i);
			REQUIRE(bh.size() == 11 - i);
			REQUIRE(bh.get(handle) == i);
		}
	}
	SECTION("push(T&&)") {
		for (int i = 10; i > 0; --i) {
			graphlib::BinHeap<int>::Handle handle = bh.push(std::move(i));
			REQUIRE(bh.top() == i);
			REQUIRE(bh.size() == 11 - i);
			REQUIRE(bh.get(handle) == i);
		}
	}
}

TEST_CASE("pop()") {
	graphlib::BinHeap<int> bh;
	bh.pop();
	REQUIRE(bh.empty());
	for (int i = 0; i < 3; ++i)
		bh.push(i);
	for (int i = 0; i < 3; ++i) {
		REQUIRE(bh.top() == i);
		bh.pop();
	}
}

TEST_CASE("emplace(Args&&...)") {
	graphlib::BinHeap<std::vector<int>> bh;
	SECTION("graphlib::BinHeap<std::vector<int>>::emplace with count argument of vector") {
		bh.emplace(5);
		REQUIRE(bh.size() == 1);
		REQUIRE(bh.top() == std::vector<int>{0, 0, 0, 0, 0});
		bh.pop();
	}
	SECTION("graphlib::BinHeap<std::vector<int>>::emplace with two integers") {
		bh.emplace(5, 3);
		REQUIRE(bh.size() == 1);
		REQUIRE(bh.top() == std::vector<int>{3, 3, 3, 3, 3});
		bh.pop();
	}
}

TEST_CASE("swap(BinHeap&)") {
	graphlib::BinHeap<int> bh_one;
	graphlib::BinHeap<int> bh_two;
	for (int i = 0; i < 9; ++i) {
		bh_one.push(i);
		bh_two.push(i + 1);
	}
	bh_one.swap(bh_two);
	for (int i = 0; i < 9; ++i) {
		REQUIRE(bh_one.top() == i + 1);
		REQUIRE(bh_two.top() == i);
		bh_one.pop();
		bh_two.pop();
	}
}

TEST_CASE("operator==/operator!=") {
	graphlib::BinHeap<int> bh;
	REQUIRE(bh == bh);
	graphlib::BinHeap<int> bh_cp(bh);
	bh.push(1);
	REQUIRE(bh != bh_cp);
	REQUIRE((bh == bh_cp) == false);
}

TEST_CASE("topHandle()") {
	graphlib::BinHeap<int> bh;
	for (int i = 10; i > 0; --i)
		bh.push(i);
	auto handle = bh.topHandle();
	REQUIRE(bh.get(handle) == 1);
}

TEST_CASE("BinHeap::swap(BinHeap&)") {
	graphlib::BinHeap<int> bh_one{1, 2, 3, 4, 5, 6, 7, 8, 9};
	graphlib::BinHeap<int> bh_two{0, 1, 2, 3, 4, 5, 6, 7, 8};
	bh_one.swap(bh_two);
	REQUIRE(bh_one != bh_two);
	REQUIRE(bh_one == graphlib::BinHeap<int>{0, 1, 2, 3, 4, 5, 6, 7, 8});
	REQUIRE(bh_two == graphlib::BinHeap<int>{1, 2, 3, 4, 5, 6, 7, 8, 9});
}

TEST_CASE("BinHeap::update(const Handle& handle, const T& value)") {
	graphlib::BinHeap<int> bh{1, 2, 3, 4, 5, 6, 7};
	auto handle = bh.push(8);
	REQUIRE(bh.get(handle) == 8);
	bh.update(handle, 0);
	REQUIRE(bh.get(handle) == 0);
}
