#include <iostream>
#include <readerwriterqueue/readerwriterqueue.h>

int main()
{
    moodycamel::ReaderWriterQueue<int, 8192>  testQueue;

    testQueue.try_enqueue( 1 );
    int a = 0;
    bool ret = testQueue.try_dequeue( a );

    std::cout << "Dequeued: " << std::to_string( a ) << std::endl;

    return 0;
}

