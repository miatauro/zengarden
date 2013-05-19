/*
 * Copyright (C) 2009  Cyril Soldani
 * 
 * This file is part of QuickCheck++.
 * 
 * QuickCheck++ is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your option)
 * any later version.
 * 
 * QuickCheck++ is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * QuickCheck++. If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * \file generate.hh
 *
 * Defines the data generator interface. This file also contains some functions
 * and templates defining generators for basic types and containers.
 *
 * \todo more generators (\eg strings and other containers than vectors)
 * \todo implement more generator helper functions
 */

#ifndef QUICKCHECK_GENERATE_H
#define QUICKCHECK_GENERATE_H

#include <algorithm>
#include <cassert>
#include <cstdlib>
#include <limits>
#include <vector>

namespace quickcheck {

/**
 * Generates a value in a range.
 *
 * \tparam A the value type (must be numeric)
 *
 * \param low  the lower bound (inclusive)
 * \param high the higher bound (inclusive)
 *
 * \return a value between \c low and \c high (both inclusive).
 */
template<class A>
A generateInRange(A low, A high)
{
   assert(low <= high);
   A offset = static_cast<A>(double(rand()) / RAND_MAX
                             * (double(high) - double(low) + 1));
   return static_cast<A>(low + offset);
}

/**
 * Chooses randomly an element out of a sequence.
 *
 * \param first an iterator pointing to first element
 * \param last  an iterator pointing to after-last element
 *
 * \return the chosen element
 */
template<class A, class Iter>
A oneOf(Iter first, Iter last)
{
   assert(std::distance(first, last) > 0);
   size_t len = size_t(std::distance(first, last));
   assert(len > 0);
   size_t index = generateInRange(size_t(0), len - 1);
   Iter i;
   for (i = first; index != 0; ++i)
      --index;
   return *i;
}

/**
 * Generates randomly an integer value. The value is taken in interval \c [0,
 * n] for unsigned values and in interval \c [-n, n] for signed values. These
 * intervals are of course clipped to the minimum and maximum values admissible
 * for the returned type.
 *
 * \tparam A the return type
 *
 * \param n the size hint
 *
 * \return a random integer in the intersection of \c [-n, n] and the A's range
 */
template<class A>
A generateInteger(size_t n)
{
   // not appropriate for floats as it chooses only integers
   assert(std::numeric_limits<A>::is_integer);
   // clips [-n, n] interval if necessary
   A max = std::numeric_limits<A>::max();
   A min = std::numeric_limits<A>::min();
   A high = (n > size_t(max)) ? max : A(n);
   A low = (n > size_t(-min)) ? min : A(-n);
   // generates integer in clipped interval
   return generateInRange(low, high);
}

/**
 * Generates a boolean value randomly. The size hint is ignored.
 *
 * \param out the reference to be set to \c true or \c false at random
 */
static inline void generate(size_t, bool& out)
{
   static const bool booleans[] = { true, false };
   out = oneOf<bool>(&booleans[0], &booleans[2]);
}

/**
 * Generates a character at random. For low values of \c n, the character is
 * alphanumeric. For intermediate values, it is ASCII and printable. For high
 * values of \c n, the generated character can have any value in its range,
 * even if it means it is not ASCII and/or not printable.
 *
 * \warning the zero character may be generated by this generator (\c n > 94)
 * \warning the zero character is not generated very often by this generator
 * (and only for \c n > 94)
 *
 * \param n the size hint, influences the \e basicness of generated character
 * \param out the reference to be set to the generated character
 */
static inline void generate(size_t n, char& out)
{
   static const char *basicChars = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789"
      "abcdefghijklmnopqrstuvwxyz";
   if (n < sizeof (basicChars) - 1)
      out = oneOf<char>(&basicChars[0], &basicChars[n]);
   else if (n < 95)
      out = generateInRange(' ', '~'); // any printable ASCII
   else
      out = generateInRange(char(0x00), char(0xff));
}

/**
 * \copydoc generate(size_t, char&)
 */
static inline void generate(size_t n, unsigned char& out)
{
   char c;
   generate(n, c);
   out = static_cast<unsigned char>(c);
}

/**
 * Generates a short integer in range \c [-n, n].
 *
 * \param n   the size hint
 * \param out the reference to be set to the generated value
 */
static inline void generate(size_t n, short& out)
{
   out = generateInteger<short>(n);
}

/**
 * Generates an unsigned short integer in range \c [0, n].
 *
 * \param n   the size hint
 * \param out the reference to be set to the generated value
 */
static inline void generate(size_t n, unsigned short& out)
{
   out = generateInteger<unsigned short>(n);
}

/**
 * Generates an integer in range \c [-n, n].
 *
 * \param n   the size hint
 * \param out the reference to be set to the generated value
 */
static inline void generate(size_t n, int& out)
{
   out = generateInteger<int>(n);
}

/**
 * Generates an unsigned integer in range \c [0, n].
 *
 * \param n   the size hint
 * \param out the reference to be set to the generated value
 */
static inline void generate(size_t n, unsigned int& out)
{
   out = generateInteger<unsigned int>(n);
}

/**
 * Generates a long integer in range \c [-n, n].
 *
 * \param n   the size hint
 * \param out the reference to be set to the generated value
 */
static inline void generate(size_t n, long& out)
{
   out = generateInteger<long>(n);
}

/**
 * Generates an unsigned long integer in range \c [0, n].
 *
 * \param n   the size hint
 * \param out the reference to be set to the generated value
 */
static inline void generate(size_t n, unsigned long& out)
{
   out = generateInteger<unsigned long>(n);
}

/**
 * Generates a single-precision floating point value in range \c [-n, n].
 *
 * \param n   the size hint
 * \param out the reference to be set to the generated value
 */
static inline void generate(size_t n, float& out)
{
   out = float(rand()) / float(RAND_MAX) * 2 * float(n) - float(n);
}

/**
 * Generates a double-precision floating point value in range \c [-n, n].
 *
 * \param n   the size hint
 * \param out the reference to be set to the generated value
 */
static inline void generate(size_t n, double& out)
{
   out = double(rand()) / RAND_MAX * 2 * double(n) - double(n);
}

/**
 * Generates a <tt>long double</tt> floating point value in range \c [-n, n].
 *
 * \param n   the size hint
 * \param out the reference to be set to the generated value
 */
static inline void generate(size_t n, long double& out)
{
   out = static_cast<long double>(rand()) / RAND_MAX * 2 * n - n;
}

/**
 * Generates a vector of 0 to \c n randomly-generated values of type \c A.
 *
 * \tparam A the element type
 *
 * \param n   the size hint
 * \param out the reference to be set to the generated value
 */
template<class A>
void generate(size_t n, std::vector<A>& out)
{
   unsigned len;
   generate(n, len);
   for (size_t i = 0; i < len; ++i) {
      A a;
      generate(n - 1, a);
      out.push_back(a);
   }
}

template <class A>
void generate(size_t n, std::pair<A, A>& p) {
  generate(n, p.first);
  generate(n, p.second);
}
}
#endif // !QUICKCHECK_GENERATE_H
