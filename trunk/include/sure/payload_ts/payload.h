/*
 * payload.h
 *
 *  Created on: 11.07.2013
 *      Author: fiolka
 */

#ifndef SURE_PAYLOAD_H_
#define SURE_PAYLOAD_H_

#include <boost/thread.hpp>

namespace sure
{

  namespace payload
  {
    typedef boost::shared_mutex shared_mutex;
    typedef boost::shared_lock<shared_mutex> shared_lock;
    typedef boost::unique_lock<shared_mutex> unique_lock;
    typedef boost::lock_guard<shared_mutex> lock_guard;
    using boost::lock;
    using boost::adopt_lock;
    using boost::defer_lock;


    class Payload
    {

      public:

        Payload() { }
        virtual ~Payload() { }

      protected:

        shared_mutex mutex_;

    };

  }
}




#endif /* SURE_PAYLOAD_H_ */
