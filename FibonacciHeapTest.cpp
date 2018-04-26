#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include "fibonacciheaplist.h"

TEST_CASE( "Fibonacci heap constructors" ) {
	SECTION("Default constructible") {
		FibHeap<int> fh;
	}
	SECTION("FibHeap(const FibHeap&)") {
		FibHeap<int> fh;
		for (int i = 0; i < 20; ++i)
		fh.push(i);
		FibHeap<int> fh_cp(fh);
		REQUIRE(fh_cp.size() == 20);
		REQUIRE(fh_cp.empty() == false);
		for (int i = 0; i < 20; ++i) {
			REQUIRE(fh_cp.top() == i);
			fh_cp.pop();
		}
	}
	SECTION("FibHeap(std::initializer_list<T>, const Compare& = Compare())") {
		FibHeap<int> fh_one{1, 2, 3, 4, 5, 6, 7, 8, 9};
		FibHeap<int, std::greater<int>> fh_two({1, 2, 3, 4, 5, 6, 7, 8, 9}, std::greater<int>());
		REQUIRE(fh_one.size() == 9);
		REQUIRE(fh_two.size() == 9);
		for (int i = 1; i < 10; ++i) {
			REQUIRE(fh_one.top() == i);
			REQUIRE(fh_two.top() == 10 - i);
			fh_one.pop();
			fh_two.pop();
		}
	}
	SECTION("FibHeap(InputIt first, InputIt last, const Compare& = Compare())") {
		std::vector<int> vec{1, 2, 3, 4, 5, 6, 7, 8, 9};
		FibHeap<int> fh(vec.begin(), vec.end(), std::less<int>());
		REQUIRE(fh.top() == 1);
		REQUIRE(fh.size() == 9);
	}
	SECTION("FibHeap(const Container& cont, const Compare& = Compare())") {
		std::vector<int> vec{1, 2, 3, 4, 5, 6, 7, 8, 9};
		FibHeap<int> fh(vec, std::less<int>());
		REQUIRE(fh.top() == 1);
		REQUIRE(fh.size() == 9);
	}
}

TEST_CASE("FibHeap copy constructor") {
	FibHeap<int> fh;
	for (int i = 0; i < 20; ++i)
		fh.push(i);
	FibHeap<int> fh_cp(fh);
	REQUIRE(fh_cp.size() == 20);
	REQUIRE(fh_cp.empty() == false);
	for (int i = 0; i < 20; ++i) {
		REQUIRE(fh_cp.top() == i);
		fh_cp.pop();
	}
}

TEST_CASE("FibHeap assignment operator") {
	FibHeap<int> fh;
	for (int i = 0; i < 20; ++i)
		fh.push(i);
	FibHeap<int> fh_cp;
	fh_cp = fh;
	REQUIRE(fh_cp.size() == 20);
	REQUIRE(fh_cp.empty() == false);
	for (int i = 0; i < 20; ++i) {
		REQUIRE(fh_cp.top() == i);
		fh_cp.pop();
	}
}

TEST_CASE("FibHeap::size") {
	FibHeap<int> fh;
	for (int i = 0; i < 30; ++i)
		fh.push(i);
	REQUIRE(fh.size() == 30);
}

TEST_CASE("FibHeap::empty") {
	FibHeap<int> fh;
	REQUIRE(fh.empty());
	for (int i = 0; i < 30; ++i)
		fh.push(i);
	REQUIRE(fh.empty() == false);
}

TEST_CASE("FibHeap::top") {
	FibHeap<int> fh;
	for (int i = 1; i < 10; ++i)
		fh.push(i);
	REQUIRE(fh.top() == 1);

	FibHeap<int, std::greater<int>> fhg;
	for (int i = 1; i < 10; ++i)
		fhg.push(i);
	REQUIRE(fhg.top() == 9);
}

TEST_CASE( "FibHeap::push") {
	FibHeap<int> fh;
	SECTION("push(const T& value)") {
		for (int i = 10; i > 0; --i) {
			fh.push(i);
			REQUIRE(fh.top() == i);
		}
	}
	SECTION("push(T&& value)") {
		for (int i = 10; i > 0; --i) {
			fh.push(std::move(i));
			REQUIRE(fh.top() == i);
		}
	}
}

TEST_CASE("FibHeap::pop") {
	FibHeap<int> fh;
	for (int i = 0; i < 10; ++i)
		fh.push(i);
	for (int i = 0; i < 10; ++i) {
		REQUIRE(fh.top() == i);
		fh.pop();
	}
}
