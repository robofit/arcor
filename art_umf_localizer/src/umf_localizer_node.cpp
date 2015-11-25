#include "umf.h"

using namespace umf;

int main(int argc, char* argv[])
{

    UMFDetector<1> *detector = new UMFDetector<1>(UMF_FLAG_ITER_REFINE|UMF_FLAG_TRACK_POS| UMF_FLAG_SUBWINDOWS | UMF_FLAG_SUBPIXEL);
    detector->setTrackingFlags(UMF_TRACK_MARKER | UMF_TRACK_SCANLINES);

    return 0;
}
