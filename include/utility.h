#ifndef BA0155BB_0E44_42FC_A3FC_AEF70B217A07
#define BA0155BB_0E44_42FC_A3FC_AEF70B217A07
inline void drawCross(Mat display_image,Point center, Scalar color, int d )
{
   line( display_image, Point( center.x - d, center.y - d ), Point( center.x + d, center.y + d ), color, 2, LINE_AA, 0);
   line( display_image, Point( center.x + d, center.y - d ), Point( center.x - d, center.y + d ), color, 2, LINE_AA, 0 );
}  


void on_mouse(int e, int x, int y, int d, void *ptr)
{
	Point*p = (Point*)ptr;
	p->x = x;
	p->y = y;
}
Eigen::Vector2d point_to_vector2d(Point p){
	Eigen::Vector2d result;
	result[0] = p.x;
	result[1] = p.y;
	return result;
}

#endif /* BA0155BB_0E44_42FC_A3FC_AEF70B217A07 */
