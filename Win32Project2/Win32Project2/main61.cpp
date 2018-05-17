
#define _CRT_SECURE_NO_WARNINGS
#include <pcl/keypoints/harris_2d.h>
#include <pcl/point_types.h>

static const struct {
	unsigned int   width;
	unsigned int   height;
	unsigned int   bytes_per_pixel; /* 2:RGB16, 3:RGB, 4:RGBA */
	unsigned char  pixel_data[9 * 9 * 3 + 1];
} gimp_image = {
	9, 9, 3,
	"\377\346\307\262\377\274\221\201\377\333\340\334\333\340\334\330\335\331"
	"\261\265\262\264\260\260\265\272\300\273\302\273\272\277\271\306\313\305"
	"\314\316\311\316\320\313\316\316\314\326\326\324\334\336\333\335\337\334"
	"\262\263\255\243\242\235\251\250\243\257\253\250\265\261\256\265\260\255"
	"\273\266\263\276\272\271\303\277\276\255\250\244\214\207\203\213\203\200"
	"\216\204\202\225\213\211\224\212\211\221\207\206\217\205\204\226\214\213"
	"\272\270\254\242\236\222\246\234\222\245\227\214\223\177v\235\202{\211lh"
	"{\\Y\202dd\317\317\305\300\276\261\274\264\251\272\256\240\263\237\224\273"
	"\240\225\246\205~\214ga\206^^\332\335\322\324\326\311\317\314\275\314\305"
	"\263\316\276\256\330\301\257\331\271\254\267\217\205\204TP\332\340\326\335"
	"\341\323\334\337\316\334\332\305\334\324\275\345\323\275\344\310\263\321"
	"\254\234\261\177v\330\335\326\334\342\330\341\345\327\337\342\315\341\337"
	"\310\343\334\300\346\325\273\346\307\262\274\221\201",
};


int main(int argc, char *argv[])
{

	typedef pcl::PointCloud<pcl::PointXYZI> PointCloudTXYZI;

	PointCloudTXYZI::Ptr cloud(new PointCloudTXYZI);
	for (int i = 0, ie = gimp_image.width * gimp_image.height * gimp_image.bytes_per_pixel; i < ie; i += gimp_image.bytes_per_pixel)
	{
		const unsigned char* r = &gimp_image.pixel_data[i];
		const unsigned char* g = &gimp_image.pixel_data[i + 1];
		const unsigned char* b = &gimp_image.pixel_data[i + 2];

		pcl::PointXYZI point;
		point.x = i / gimp_image.bytes_per_pixel - i / (gimp_image.width*gimp_image.bytes_per_pixel)*gimp_image.width;
		point.y = i / (gimp_image.width*gimp_image.bytes_per_pixel);
		point.z = 1;

		point.intensity = (299 * (int)*r + 587 * (int)*g + 114 * (int)*b) * 0.001f;
		cloud->push_back(point);
	}

	cloud->is_dense = true;
	cloud->height = gimp_image.height;
	cloud->width = gimp_image.width;

	int winWidth = 3;
	int winHeight = 3;
	int minDistance = 5;
	float threshold = 0;
	bool suppression = true;
	int method = 1;
	float radius = 10;

	for (int i = 0, ie = 50; i < ie; ++i)
	{
		PointCloudTXYZI* out = new PointCloudTXYZI;
		pcl::HarrisKeypoint2D<pcl::PointXYZI, pcl::PointXYZI> harris((pcl::HarrisKeypoint2D<pcl::PointXYZI, pcl::PointXYZI>::ResponseMethod)method, winWidth, winHeight, minDistance, threshold);
		harris.setRadiusSearch(radius);
		harris.setNumberOfThreads(1);
		harris.setInputCloud(cloud);
		harris.setNonMaxSupression(suppression);
		harris.setRefine(false);
		harris.compute(*out);
		printf("run: %i, in size: %lu, out size :%lu \n", i, cloud->size(), out->size());
		//delete out; //The result cloud is not deleted as this will increase the non-deterministic behaviour. 
	}
}