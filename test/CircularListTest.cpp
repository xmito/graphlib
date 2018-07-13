#include "CircularList.h"
#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include <algorithm>

TEST_CASE("Pushing to Circular list") {
	CircularList<int> circ;
	SECTION("push_front") {
		for(int i = 0; i < 20; ++i)
			circ.push_front(i);
		REQUIRE(circ.size() == 20);
		int counter = 19;
		for (auto& i : circ)
			REQUIRE(i == counter--);
	}
	SECTION("push_back()") {
		for(int i = 0; i < 20; ++i)
			circ.push_back(i);
		REQUIRE(circ.size() == 20);
		int counter = 0;
		for (auto& i : circ)
			REQUIRE(i == counter++);
	}
	SECTION("combined push_back & push_front") {
		for(int i = 10; i < 20; ++i)
			circ.push_back(i);
		for(int i = 9; i >= 0; --i)
			circ.push_front(i);
		REQUIRE(circ.size() == 20);
		int counter = 0;
		for (auto& i : circ)
			REQUIRE(i == counter++);
	}
}

TEST_CASE("Popping from Circular list") {
	CircularList<int> circ;
	for (int i = 0; i < 20; ++i)
		circ.push_back(i);
	REQUIRE(circ.front() == 0);
	REQUIRE(circ.back() == 19);
	SECTION("pop_front") {
		circ.pop_front();
		REQUIRE(circ.front() == 1);
		REQUIRE(circ.back() == 19);
		REQUIRE(circ.size() == 19);
	}
	SECTION("pop_back") {
		circ.pop_back();
		REQUIRE(circ.back() == 18);
		REQUIRE(circ.front() == 0);
		REQUIRE(circ.size() == 19);
	}
}

TEST_CASE("clear Circular list") {
	CircularList<int> circ;
	for (int i = 0; i < 20; ++i)
		circ.push_back(i);
	circ.clear();
	REQUIRE(circ.size() == 0);
	REQUIRE(circ.empty());
	auto bit = circ.begin();
	auto eit = circ.end();
	REQUIRE(bit == eit);
	// Can push after clear
	for(int i = 10; i < 20; ++i)
		circ.push_back(i);
	for(int i = 9; i >= 0; --i)
		circ.push_front(i);
	REQUIRE(circ.size() == 20);
	int counter = 0;
	for (auto& i : circ)
		REQUIRE(i == counter++);
	REQUIRE(circ.front() == 0);
	REQUIRE(circ.back() == 19);
}

TEST_CASE("front & back") {
	CircularList<int> circ;
	SECTION("front()") {
		circ.push_front(0);
		REQUIRE(circ.front() == 0);
	}
	SECTION("back()") {
		circ.push_back(1);
		REQUIRE(circ.back() == 1);
	}
	SECTION("front() & back()") {
		circ.push_back(0);
		REQUIRE(circ.front() == circ.back());
		circ.push_back(1);
		REQUIRE(circ.front() != circ.back());
	}
}

TEST_CASE("size()") {
	CircularList<int> circ;
	REQUIRE(circ.size() == 0);
	circ.push_back(1);
	REQUIRE(circ.size() == 1);
}

TEST_CASE("empty()") {
	CircularList<int> circ;
	REQUIRE(circ.empty());
	circ.push_front(0);
	REQUIRE_FALSE(circ.empty());
}

TEST_CASE("begin & end") {
	CircularList<int> circ;
	REQUIRE(circ.begin() == circ.end());
	circ.push_back(0);
	REQUIRE(*circ.begin() == 0);
	REQUIRE(circ.begin() != circ.end());
	auto it = circ.end();
	--it;
	REQUIRE(it == circ.begin());
}

TEST_CASE("Iterator") {
	CircularList<int> circ;
	circ.push_back(0);
	SECTION("operator++/--") {
		auto it1 = circ.begin();
		++it1;
		REQUIRE(circ.end() == it1);

		it1 = circ.begin();
		auto it2 = it1++;
		REQUIRE(circ.begin() == it2);
		REQUIRE(circ.end() == it1);

		auto it3 = circ.end();
		--it3;
		REQUIRE(circ.begin() == it3);
		
		it3 = circ.end();
		auto it4 = it3--;
		REQUIRE(circ.begin() == it3);
		REQUIRE(circ.end() == it4);
	}
	SECTION("Dereferencing") {
		REQUIRE(*circ.begin() == 0);
	}
	SECTION("Dereferencing end node with LastAction::DEC") {
		circ.push_back(1);
		auto it = --circ.begin();
		auto backit = --circ.end();
		REQUIRE(*it == *backit);
	}
	SECTION("Dereferencing end node with LastAction::INC") {
		circ.push_back(1);
		auto it = ++(--circ.end());
		REQUIRE(*circ.begin() == *it);
	}
	SECTION("Construct iterator from iterator") {
		CircularList<int>::iterator it = circ.begin();
		CircularList<int>::iterator nit(it);
		REQUIRE(it == nit);
		it = circ.end();
		nit = it;
		REQUIRE(it == nit);
	}
	SECTION("Construct const_iterator from const_iterator") {
		CircularList<int>::const_iterator cit = circ.cbegin();
		CircularList<int>::const_iterator it(cit);
		REQUIRE(cit == it);
		cit = circ.cend();
		it = cit;
		REQUIRE(it == cit);
	}
	SECTION("Construct const_iterator from iterator") {
		CircularList<int>::iterator it = circ.begin();
		CircularList<int>::const_iterator cit(it);
		REQUIRE(it == cit);
		it = circ.end();
		cit = it;
		REQUIRE(cit == it);
	}
	SECTION("operator->") {
		typedef struct {
			int one;
			int two;
		} str_t;
		CircularList<str_t> circstruct;
		circstruct.push_back({1, 2});
		CircularList<str_t>::iterator beg = circstruct.begin();
		REQUIRE(beg->one == 1);
		REQUIRE(beg->two == 2);
	}
}

TEST_CASE("Reverse iterators") {
	CircularList<int> circ;
	for (int i = 0; i < 20; ++i)
		circ.push_back(i);
	SECTION("rbegin/rend") {
		auto bit = circ.rbegin();
		auto eit = circ.rend();
		for (int i = 19; i >= 0; --i) {
			REQUIRE(*bit == i);
			++bit;
		}
		REQUIRE(bit == eit);
	}
	SECTION("crbegin/crend") {
		auto cbit = circ.crbegin();
		auto ceit = circ.crend();
		for (int i = 19; i >= 0; --i) {
			REQUIRE(*cbit == i);
			++cbit;
		}
		REQUIRE(cbit == ceit);
	}
	SECTION("Dereferencing") {
		REQUIRE(*circ.rbegin() == 19);
	}
	SECTION("Dereferencing end node with LastAction::NOA") {
		REQUIRE(*circ.rbegin() == *circ.rend());
	}
	SECTION("Dereferencing end node with LastAction::DEC") {
		auto it = --circ.rbegin();
		auto backit = --circ.rend();
		REQUIRE(*it == *backit);
	}
	SECTION("Dereferencing end node with LastAction::INC") {
		auto it = ++(--circ.rend());
		REQUIRE(*circ.rbegin() == *it);
	}
	SECTION("Construct reverse_iterator from reverse_iterator") {
		CircularList<int>::reverse_iterator it = circ.rbegin();
		CircularList<int>::reverse_iterator nit(it);
		REQUIRE(it == nit);
		it = circ.rend();
		nit = it;
		REQUIRE(it == nit);
	}
	SECTION("Construct const_reverse_iterator from const_reverse_iterator") {
		CircularList<int>::const_reverse_iterator cit = circ.crbegin();
		CircularList<int>::const_reverse_iterator it(cit);
		REQUIRE(cit == it);
		cit = circ.crend();
		it = cit;
		REQUIRE(it == cit);
	}
	SECTION("Construct const_reverse_iterator from reverse_iterator") {
		CircularList<int>::reverse_iterator it = circ.rbegin();
		CircularList<int>::const_reverse_iterator cit(it);
		REQUIRE(it == cit);
		it = circ.rend();
		cit = it;
		REQUIRE(cit == it);
	}
	SECTION("Construct reverse_iterator from iterator") {
		auto it = circ.begin();
		CircularList<int>::reverse_iterator rit(it);
	}
	SECTION("Construct const_reverse_iterator from iterator") {
		auto it = circ.begin();
		CircularList<int>::const_reverse_iterator rit(it);
	}
	SECTION("Construct const_reverse_iterator from const_iterator") {
		auto it = circ.cbegin();
		CircularList<int>::const_reverse_iterator rit(it);
	}
	SECTION("base() method") {
		CircularList<int>::iterator it = circ.begin();
		CircularList<int>::reverse_iterator rit(it);
		REQUIRE(rit.base() == it);
	}
}

TEST_CASE("Move ctor/assignment") {
	CircularList<int> circ;
	for (int i = 0; i < 20; ++i)
		circ.push_back(i);
	REQUIRE(circ.size() == 20);
	int counter = 0;
	SECTION("Move ctor") {
		CircularList<int> circ_mv(std::move(circ));
		for (auto i : circ_mv)
			REQUIRE(i == counter++);
		REQUIRE(counter == 20);
		REQUIRE(circ_mv.size() == 20);
		REQUIRE(circ_mv.front() == 0);
		REQUIRE(circ_mv.back() == 19);
		REQUIRE(circ.size() == 0);
		REQUIRE(circ.empty());
		REQUIRE(circ.begin() == circ.end());
	}

	SECTION("Move assignment") {
		CircularList<int> circ_mv;
		circ_mv = std::move(circ);
		for (auto i : circ_mv)
			REQUIRE(i == counter++);
		REQUIRE(counter == 20);
		REQUIRE(circ_mv.size() == 20);
		REQUIRE(circ_mv.front() == 0);
		REQUIRE(circ_mv.back() == 19);
		REQUIRE(circ.size() == 0);
		REQUIRE(circ.empty());
		REQUIRE(circ.begin() == circ.end());
	}
}

TEST_CASE("Copy ctor/assignment") {
	CircularList<int> circ;
	for (int i = 0; i < 20; ++i)
		circ.push_back(i);
	REQUIRE(circ.size() == 20);
	SECTION("Copy ctor") {
		CircularList<int> circ_cp = circ;
		REQUIRE(circ_cp == circ);
		REQUIRE(circ_cp.size() == circ.size());
		REQUIRE(circ_cp.front() == circ.front());
		REQUIRE(circ_cp.back() == circ.back());
		REQUIRE(circ_cp.empty() == circ.empty());
		REQUIRE(circ_cp.begin() != circ_cp.end());
	}
	SECTION("Copy assignment") {
		CircularList<int> circ_cp;
		circ_cp = circ;
		REQUIRE(circ_cp == circ);
		REQUIRE(circ_cp.size() == circ.size());
		REQUIRE(circ_cp.front() == circ.front());
		REQUIRE(circ_cp.back() == circ.back());
		REQUIRE(circ_cp.empty() == circ.empty());
		REQUIRE(circ_cp.begin() != circ_cp.end());
	}
}

TEST_CASE("Inserting into CircularList") {
	CircularList<int> circ;
	typename CircularList<int>::const_iterator cit = circ.cbegin();
	SECTION("Copy one into position") {
		int i = 2;
		circ.insert(cit, i);
		REQUIRE(circ.front() == i);
		REQUIRE(circ.size() == 1);
		i = 3;
		cit = circ.cbegin();
		circ.insert(cit, i);
		REQUIRE(circ.front() == i);
		REQUIRE(circ.size() == 2);
	}
	SECTION("Move one into position") {
		circ.insert(cit, 2);
		REQUIRE(circ.front() == 2);
		REQUIRE(circ.size() == 1);
		cit = circ.cbegin();
		circ.insert(cit, 3);
		REQUIRE(circ.front() == 3);
		REQUIRE(circ.size() == 2);
	}
	SECTION("Insert count values into position") {
		circ.insert(cit, 20u, 2);
		REQUIRE(circ.size() == 20);
		for (auto val : circ)
			REQUIRE(val == 2);
	}
	SECTION("Insert range into position") {
		std::vector<int> vec{1, 2, 3, 4, 5, 6};
		circ.insert(cit, vec.begin(), vec.end());
		REQUIRE(circ.size() == vec.size());
		REQUIRE(std::equal(circ.begin(), circ.end(), vec.begin(), vec.end()));
	}
	SECTION("Insert initializer_list into position") {
		circ.insert(cit, {1, 2, 3, 4, 5, 6});
		REQUIRE(circ.size() == 6);
		int counter = 1;
		for (auto val : circ)
			REQUIRE(val == counter++);
	}
}

TEST_CASE("Assign method") {
	CircularList<int> circ_empty;
	CircularList<int> circ;
	for (int i = 0; i < 20; ++i)
		circ.push_back(i);
	SECTION("Assign range") {
		std::vector<int> vec{1, 2, 3, 4, 5, 6};
		circ.assign(vec.begin(), vec.end());
		REQUIRE(circ.size() == vec.size());
		REQUIRE(circ.front() == 1);
		REQUIRE(circ.back() == 6);

		circ_empty.assign(vec.begin(), vec.end());
		REQUIRE(circ.size() == vec.size());
		REQUIRE(circ.front() == 1);
		REQUIRE(circ.back() == 6);
	}
	SECTION("Assign initializer_list") {
		circ.assign({1, 2, 3, 4, 5, 6});
		REQUIRE(circ.size() == 6);
		REQUIRE(circ.front() == 1);
		REQUIRE(circ.back() == 6);

		circ_empty.assign({1, 2, 3, 4, 5, 6});
		REQUIRE(circ.size() == 6);
		REQUIRE(circ.front() == 1);
		REQUIRE(circ.back() == 6);
	}
}

TEST_CASE("Emplace_back & emplace_front") {
	CircularList<std::vector<int>> circ;
	circ.push_back(std::vector<int>(10, 10));
	SECTION("emplace_back") {
		circ.emplace_back(20, 10);
		REQUIRE(circ.size() == 2);
		REQUIRE(circ.back() == std::vector<int>(20, 10));
		REQUIRE(circ.front() != circ.back());
	}
	SECTION("emplace_front") {
		circ.emplace_front(20, 10);
		REQUIRE(circ.size() == 2);
		REQUIRE(circ.front() == std::vector<int>(20, 10));
		REQUIRE(circ.front() != circ.back());
	}
}

TEST_CASE("Resize list") {
	SECTION("resize list to have smaller number of elements") {
		CircularList<int> circ;
		for (int i = 0; i < 20; ++i)
			circ.push_back(i);
		circ.resize(10);
		REQUIRE(circ.size() == 10);
		REQUIRE(circ.front() == 0);
		REQUIRE(circ.back() == 9);

		circ.resize(5, 2);
		REQUIRE(circ.size() == 5);
		REQUIRE(circ.front() == 0);
		REQUIRE(circ.back() == 4);
	}
	SECTION("resize list with default constructed value") {
		CircularList<int> circ;
		circ.resize(10);
		REQUIRE(circ.size() == 10);
		REQUIRE(circ.front() == circ.back());
		REQUIRE(circ.front() == 0);
	}
	SECTION("resize list with custom value") {
		CircularList<int> circ;
		circ.resize(10, int());
		REQUIRE(circ.size() == 10);
		REQUIRE(circ.front() == circ.back());
		REQUIRE(circ.front() == 0);
	}
}

TEST_CASE("Erasing from list") {
	CircularList<int> circ;
	for (int i = 0; i < 20; ++i)
		circ.push_back(i);
	SECTION("Erasing single element at position") {
		CircularList<int>::iterator retit = circ.erase(circ.begin());
		REQUIRE(retit == circ.begin());
		REQUIRE(circ.size() == 19);
		REQUIRE(circ.front() == 1);
		REQUIRE(circ.back() == 19);

		retit = circ.erase(--circ.end());
		REQUIRE(retit == circ.end());
		REQUIRE(circ.size() == 18);
		REQUIRE(circ.back() == 18);
		REQUIRE(circ.front() == 1);

		CircularList<int>::const_iterator it = circ.begin();
		for (int i = 0; i < 10; ++i)
			++it;
		retit = circ.erase(it);
		REQUIRE(circ.size() == 17);
		REQUIRE(*retit == 12);
		REQUIRE(*(--retit) == 10);
	}
	SECTION("Erasing a rage of elements") {
		CircularList<int>::iterator bit = circ.begin();
		CircularList<int>::iterator eit = bit;
		++eit;
		++eit;
		CircularList<int>::iterator retit = circ.erase(bit, eit);
		REQUIRE(retit == eit);
		REQUIRE(circ.size() == 18);
		REQUIRE(circ.front() == 2);
		REQUIRE(circ.back() == 19);

		retit = circ.erase(circ.begin(), circ.end());
		REQUIRE(circ.size() == 0);
		REQUIRE(circ.begin() == circ.end());
		REQUIRE(retit == circ.end());
	}
}

TEST_CASE("Find method") {
	CircularList<int> circ;
	for (int i = 0; i < 20; ++i)
		circ.push_back(i);
	SECTION("Non-const find") {
		CircularList<int>::iterator it = circ.find(10);
		REQUIRE(*it == 10);
		it = circ.find(21);
		REQUIRE(it == circ.end());
	}
	SECTION("Const find") {
		const CircularList<int> &ccirc = const_cast<const CircularList<int>&>(circ);
		CircularList<int>::const_iterator cit = ccirc.find(10);
		REQUIRE(*cit == 10);
		cit = ccirc.find(21);
		REQUIRE(cit == ccirc.end());
	}
}

TEST_CASE("Operators") {
	CircularList<int> circa;
	CircularList<int> circb;
	CircularList<int> circz;
	for (int i = 1; i < 20; ++i) {
		circa.push_back(i - 1);
		circb.push_back(i);
	}
	circb.push_back(20);
	for (int i = 0; i < 20; ++i)
		circz.push_back(0);
	SECTION("operator==") {
		REQUIRE(circa == circa);
	}
	SECTION("operator!=") {
		REQUIRE(circa != circb);
	}
	SECTION("operator<") {
		REQUIRE(circa < circb);
		REQUIRE(!(circz < circa));
		REQUIRE(circz < circb);
	}
	SECTION("operator>") {
		REQUIRE(circb > circa);
		REQUIRE(!(circa > circz));
		REQUIRE(circb > circz);
	}
	SECTION("operator<=") {
		REQUIRE(circa <= circb);
		REQUIRE(circz <= circa);
		REQUIRE(circz <= circb);
	}
	SECTION("operator>=") {
		REQUIRE(circb >= circa);
		REQUIRE(circa >= circz);
		REQUIRE(circb >= circz);
	}
}

TEST_CASE("Merge") {
	CircularList<int> circa;
	CircularList<int> circb;
	for (int i = 0; i < 20; ++i) {
		circa.push_back(i);
		circb.push_back(i + 20);
	}
	CircularList<int> circc(circa);
	circa.merge(circb);
	REQUIRE(circa.size() == 40);
	int counter = 0;
	for (auto& val : circa)
		REQUIRE(val == counter++);
	
	circc.merge(std::move(circb));
	REQUIRE(circb.size() == 0);
	//REQUIRE(circb.begin() == circb.end());
	REQUIRE(circc.size() == 40);
	counter = 0;
	for (auto& val : circc)
		REQUIRE(val == counter++);
}

TEST_CASE("Splice") {
	CircularList<int> circa;
	CircularList<int> circb;
	for (int i = 0; i < 10; ++i) {
		circa.push_back(i);
		circb.push_back(i + 1);
	}
	SECTION("splice(const_iterator, const CircularList&, const_iterator)") {
		circa.splice(circa.begin(), circb, circb.begin());
		REQUIRE(circa.size() == 11);
		REQUIRE(circb.size() == 9);
		int counter = 2;
		for (auto& val : circb)
			REQUIRE(val == counter++);
		auto it = circa.begin();
		REQUIRE(*it == 1);
		REQUIRE(*++it == 0);
		REQUIRE(*++it == 1);
	}
	SECTION("splice(const_iterator, const CiruclarList&, const_iterator) sentinel node") {
		circa.splice(circa.begin(), circb, circb.end());
		REQUIRE(circa.size() == 10);
		REQUIRE(circb.size() == 10);
		auto ita = circa.begin();
		auto itb = circb.begin();
		for(int i = 0; ita != circa.end(); ++ita, ++itb, ++i) {
			REQUIRE(*ita == i);
			REQUIRE(*itb == i + 1);
		}
	}
}
