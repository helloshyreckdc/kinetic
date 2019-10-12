#include <ros/ros.h>
#include <visp3/gui/vpDisplayD3D.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_viewer");

    ros::NodeHandle node;

    if (argc != 2) {
        printf("Usage: %s <image name.[pgm,ppm,jpeg,png,tiff,bmp,ras,jp2]>\n", argv[0]);
        return -1;
    }
    vpImage<vpRGBa> I;
    try {
        vpImageIo::read(I, argv[1]);
    } catch (...) {
        std::cout << "Cannot read image \"" << argv[1] << "\"" << std::endl;
        return -1;
    }
    try {
#if defined(VISP_HAVE_X11)
        vpDisplayX d(I, vpDisplay::SCALE_AUTO);
#elif defined(VISP_HAVE_GDI)
        vpDisplayGDI d(I, vpDisplay::SCALE_AUTO);
#elif defined(VISP_HAVE_OPENCV)
        vpDisplayOpenCV d(I, vpDisplay::SCALE_AUTO);
#elif defined(VISP_HAVE_GTK)
        vpDisplayGTK d(I, vpDisplay::SCALE_AUTO);
#elif defined(VISP_HAVE_D3D9)
        vpDisplayD3d d(I, vpDisplay::SCALE_AUTO);
#else
        std::cout << "No image viewer is available..." << std::endl;
#endif
        vpDisplay::setTitle(I, "My image");
        vpDisplay::display(I);
        vpDisplay::flush(I);
        std::cout << "A click to quit..." << std::endl;
        vpDisplay::getClick(I);
    } catch (const vpException &e) {
        std::cout << "Catch an exception: " << e << std::endl;
    }
}
