/** @file
	@brief Header

	@versioninfo@

	@date 2011

	@author
	Ryan Pavlik
	<rpavlik@iastate.edu> and <abiryan@ryand.net>
	http://academic.cleardefinition.com/
	Iowa State University Virtual Reality Applications Center
	Human-Computer Interaction Graduate Program
*/

//          Copyright Iowa State University 2011.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#pragma once
#ifndef INCLUDED_EigenMatrixSerialize_h_GUID_3ebee56a_057c_4186_9e5a_b8efbae15236
#define INCLUDED_EigenMatrixSerialize_h_GUID_3ebee56a_057c_4186_9e5a_b8efbae15236

// Internal Includes
// - none

// Library/third-party includes
#include <Eigen/Core>
#include <boost/serialization/array.hpp>

// Standard includes
// - none

namespace boost {
	namespace serialization {

		template<class Archive, class Scalar, int RowsAtCompileTime, int ColsAtCompileTime>
		void serialize(Archive & ar, ::Eigen::Matrix<Scalar, RowsAtCompileTime, ColsAtCompileTime> & m, const unsigned int /*version*/) {
			ar & boost::serialization::make_array(m.data(), RowsAtCompileTime * ColsAtCompileTime);
		}

	} // end of namespace serialization
} // end of namespace boost
#endif // INCLUDED_EigenMatrixSerialize_h_GUID_3ebee56a_057c_4186_9e5a_b8efbae15236
