#include <iostream>
#include <concurrentqueue/concurrentqueue.h>
#include <concurrentqueue/blockingconcurrentqueue.h>

int main()
{
    int item = 0;
    
    // ConcurrentQueue
    moodycamel::ConcurrentQueue<int> q;
    q.enqueue(1);
    bool found = q.try_dequeue(item);
    assert(found && item == 1);

    // BlockingConcurrentQueue
    moodycamel::BlockingConcurrentQueue<int> bq;
    bq.enqueue( 2 );
    bq.wait_dequeue( item );
    assert( item == 2 );

    return 0;
}

