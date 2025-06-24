/**
 * @file Debug.h
 * @brief Debug 工具, 直接使用 debug(...) 即可
 * @author Cao Xin, From tourist's header.
 * @date 2025-05-28
 */

#pragma once

#include <iterator>
#include <iostream>
#include <string>
#include <utility>
#include <vector>
#include <set>
#include <map>
#include <type_traits>
#include <sstream>

using namespace std;

// --- to_string for built-ins ---
string to_string(const string &s);
string to_string(const char *s);
string to_string(char c);
string to_string(bool x);

template<class A, class B>
string to_string(const pair<A, B> &p);

// --- to_string for containers (only if std::begin/end are valid) ---
template<class A>
auto to_string(const A &v)
  -> decltype(std::begin(v), std::end(v), string())
{
    string res = "{";
    bool first = true;
    for (auto it = std::begin(v), ed = std::end(v); it != ed; ++it) {
        if (!first) res += ", ";
        first = false;
        res += to_string(*it);
    }
    res += "}";
    return res;
}

// --- fallback to_string for anything streamable ---
template<class T>
auto to_string(const T &x)
  -> decltype(declval<ostream&>() << x, string())
{
    ostringstream oss;
    oss << x;
    return oss.str();
}

// --- debug_out declarations ---
void debug_out();

template<class H, class... T>
void debug_out(const H &h, const T &... t);

// --- debug macro ---
#define debug(...) \
    cerr << "[" << #__VA_ARGS__ << "]:", debug_out(__VA_ARGS__)

// === inline/template definitions ===

template<class A, class B>
string to_string(const pair<A, B> &p) {
    return "(" + to_string(p.first) + ", " + to_string(p.second) + ")";
}

inline string to_string(const string &s) {
    return '"' + s + '"';
}

inline string to_string(const char *s) {
    return to_string(string(s));
}

inline string to_string(char c) {
    return "'" + string(1, c) + "'";
}

inline string to_string(bool x) {
    return x ? "true" : "false";
}

template<class H, class... T>
void debug_out(const H &h, const T &... t) {
    cerr << " " << to_string(h);
    debug_out(t...);
}
