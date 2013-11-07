/*
 * range.cpp
 *
 *  Created on: 20.06.2013
 *      Author: fiolka
 */

#include<sure/access/region.h>

std::ostream& sure::access::operator<<(std::ostream& stream, const Region& rhs)
{
  return stream << rhs.min() << "|" << rhs.max();
}
