#include <opencv2/opencv.hpp>
#include <math.h>
#include <vector>
#include <UV_detector.h>
#include <kalman_filter.h>
#include <algorithm>    // std::max
using namespace std;
using namespace cv;

// Vbox

Vbox::Vbox()
{
    this->id_v = 0;
    this->toppest_parent_id_v = 0;
    this->bb_v = Rect(Point2f(0, 0), Point2f(0, 0));
}

Vbox::Vbox(int seg_id_v, int col, int left, int right)
{
    this->id_v = seg_id_v;
    this->toppest_parent_id_v = seg_id_v;
    this->bb_v = Rect(Point2f(col,left), Point2f(col,right));
}

Vbox merge_two_Vbox(Vbox father, Vbox son)
{
    // merge the bounding box
    int top = (father.bb_v.tl().y < son.bb_v.tl().y) ? father.bb_v.tl().y : son.bb_v.tl().y;
    int left = (father.bb_v.tl().x < son.bb_v.tl().x) ? father.bb_v.tl().x : son.bb_v.tl().x;
    int bottom = (father.bb_v.br().y > son.bb_v.br().y) ? father.bb_v.br().y : son.bb_v.br().y;
    int right = (father.bb_v.br().x > son.bb_v.br().x) ? father.bb_v.br().x : son.bb_v.br().x;
    father.bb_v = Rect(Point2f(left, top), Point2f(right, bottom));
    return father;


    /*int top = (father.bb.tl().y < son.bb.tl().y) ? father.bb.tl().y : son.bb.tl().y;
    int left = (father.bb.tl().x < son.bb.tl().x) ? father.bb.tl().x : son.bb.tl().x;
    int bottom = (father.bb.br().y > son.bb.br().y) ? father.bb.br().y : son.bb.br().y;
    int right = (father.bb.br().x > son.bb.br().x) ? father.bb.br().x : son.bb.br().x;
    father.bb = Rect(Point2f(left, top), Point2f(right, bottom));*/

    /*int top = (father.bb_v.tl().x < son.bb_v.tl().x) ? father.bb_v.tl().x : son.bb_v.tl().x;
    int left = (father.bb_v.tl().x < son.bb_v.tl().x) ? father.bb_v.tl().x : son.bb_v.tl().x;
    int bottom = (father.bb_v.br().y > son.bb_v.br().y) ? father.bb_v.br().y : son.bb_v.br().y;
    int right = (father.bb_v.br().y > son.bb_v.br().y) ? father.bb_v.br().y : son.bb_v.br().y;
    father.bb_v = Rect(Point2f(left, top), Point2f(right, bottom));
    return father;*/
    
}

// Vtracker

Vtracker::Vtracker()
{
    this->overlap_threshold = 0.51;
}

void Vtracker::read_bb_v(vector<Rect> now_bb_v)
{
    // measurement history
    this->pre_history = this->now_history;
    this->now_history.clear();
    this->now_history.resize(now_bb_v.size());
    // kalman filters
    this->pre_filter = this->now_filter;
    this->now_filter.clear();
    this->now_filter.resize(now_bb_v.size());
    // bounding box
    this->pre_bb_v = this->now_bb_v;
    this->now_bb_v = now_bb_v;
}

void Vtracker::check_status()
{
    for (int now_id_v = 0; now_id_v < this->now_bb_v.size(); now_id_v++)
    {
        bool tracked = false;
        for (int pre_id_v = 0; pre_id_v < this->pre_bb_v.size(); pre_id_v++)
        {
            Rect overlap = this->now_bb_v[now_id_v] & this->pre_bb_v[pre_id_v];
            if (min(overlap.area() / float(this->now_bb_v[now_id_v].area()), overlap.area() / float(this->now_bb_v[now_id_v].area())) >= this->overlap_threshold)
            {
                tracked = true;
                // add current detection to history
                this->now_history[now_id_v] = this->pre_history[pre_id_v];
                this->now_history[now_id_v].push_back(Point2f(this->now_bb_v[now_id_v].x + 0.5 * this->now_bb_v[now_id_v].width, this->now_bb_v[now_id_v].y + 0.5 * this->now_bb_v[now_id_v].height));
                // add measurement to previous filter
                this->now_filter[now_id_v] = this->pre_filter[pre_id_v];
                MatrixXd z(4, 1); // measurement
                z << this->now_bb_v[now_id_v].x + 0.5 * this->now_bb_v[now_id_v].width, this->now_bb_v[now_id_v].y + 0.5 * this->now_bb_v[now_id_v].height, this->now_bb_v[now_id_v].width, this->now_bb_v[now_id_v].height;
                MatrixXd u(1, 1); // input
                u << 0;
                // run the filter 
                this->now_filter[now_id_v].estimate(z, u);
                break;
            }
        }
        if (!tracked)
        {
            // add current detection to history
            this->now_history[now_id_v].push_back(Point2f(this->now_bb_v[now_id_v].x + 0.5 * this->now_bb_v[now_id_v].width, this->now_bb_v[now_id_v].y + 0.5 * this->now_bb_v[now_id_v].height));
            // initialize filter
            int f = 30; // Hz
            double ts = 1.0 / f; // s

            // model for center filter
            double e_p = 0.4;
            double e_ps = 0.0;
            double e_m = 0.3;
            double e_ms = 0.99;
            MatrixXd A(6, 6);
           /* A << 1, 0, ts, 0, 0, 0,
                0, 1, 0, ts, 0, 0,
                0, 0, 1, 0, 0, 0,
                0, 0, 0, 1, 0, 0,
                0, 0, 0, 0, 1, 0,
                0, 0, 0, 0, 0, 1;*/
	    A << 1,0,0,0,0,0,0,1,0,0,0,0,ts,0,1,0,0,0,0,ts,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1;
            MatrixXd B(6, 1);
            B << 0, 0, 0, 0, 0, 0;
            MatrixXd H(4, 6);
            H << 1, 0, 0, 0, 0, 0,
                0, 1, 0, 0, 0, 0,
                0, 0, 0, 0, 1, 0,
                0, 0, 0, 0, 0, 1;
 	   /*H << 1, 0, 0, 0, 0, 0,
                0, 1, 0, 0, 0, 0,
                0, 0, 1, 0, 0, 0,
                0, 0, 0, 1, 0, 0;*/
            MatrixXd P = MatrixXd::Identity(6, 6) * e_m;
            P(4, 4) = e_ms; P(5, 5) = e_ms;
            MatrixXd Q = MatrixXd::Identity(6, 6) * e_p;
            Q(4, 4) = e_ps; Q(5, 5) = e_ps;
            MatrixXd R = MatrixXd::Identity(4, 4) * e_m;

            // filter initialization
            MatrixXd states(6, 1);
            states << this->now_bb_v[now_id_v].x + 0.5 * this->now_bb_v[now_id_v].width, this->now_bb_v[now_id_v].y + 0.5 * this->now_bb_v[now_id_v].height, 0, 0, this->now_bb_v[now_id_v].width, this->now_bb_v[now_id_v].height;
            kalman_filter my_filter;

            this->now_filter[now_id_v].setup(states,
                A,
                B,
                H,
                P,
                Q,
                R);
        }
    }
}

// Vdetector

Vdetector::Vdetector()
{
    this->row_downsample = 5;
    this->col_scale = 0.5;
    this->min_dist = 10;
    this->max_dist = 2000; //5000
    this->threshold_point = 2;// 2
    this->threshold_line = 6;//2
    this->min_length_line = 8;
    this->show_bounding_box_U = true;
    this->fx = 383.91;
    this->fy = 383.91;
    this->px = 318.27;
    this->py = 242.18;

    // maybe l515
    // this->fx = 228.7480;
     //this->fy = 229.0078;
    //this->px = 159.3047;
    //this->py = 123.9873;
}

void Vdetector::readdata(Mat depth)
{
    this->depth = depth;
}

void Vdetector::extract_U_map()
{
    // rescale depth map
    Mat depth_rescale;
    
    resize(this->depth, depth_rescale, Size(), 1, this->col_scale);
    Mat depth_low_res_temp = Mat::zeros(depth_rescale.rows, depth_rescale.cols, CV_8UC1);

    // construct the mask
    Rect mask_depth;
    
    uint8_t histSize = this->depth.cols / this->row_downsample;
    uint8_t bin_width = ceil((this->max_dist - this->min_dist) / float(histSize));
    // initialize U map
    
this->U_map = Mat::zeros(depth_rescale.rows, histSize,  CV_8UC1);
for (int row = 0; row < depth_rescale.rows; row++)
    {
        for (int col = 0; col < depth_rescale.cols; col++)
        {
            if (depth_rescale.at<unsigned short>(row, col) > this->min_dist && depth_rescale.at<unsigned short>(row, col) < this->max_dist)
            {
                uint8_t bin_index = (depth_rescale.at<unsigned short>(row, col) - this->min_dist) / bin_width;
                depth_low_res_temp.at<uchar>(row, col) = bin_index;
                if (this->U_map.at<uchar>(row,bin_index) < 255)
                {
                    this->U_map.at<uchar>(row,bin_index)++;
                }
            }
        }
    }

    this->depth_low_res = depth_low_res_temp;

    // smooth the U map
    GaussianBlur(this->U_map, this->U_map, Size(5, 11), 3, 10);
    cout << "histSize=" << endl;
    cout << histSize << endl;
}

void Vdetector::extract_bb_v()
{
    // initialize a mask
    //vector<vector<int> > mask(this->U_map.cols, vector<int>(this->U_map.rows, 0));
vector<vector<int> > mask(this->U_map.rows, vector<int>(this->U_map.cols, 0));
    // initialize parameters
    int u_min = this->threshold_point * this->row_downsample;
    int sum_line = 0;
    int max_line = 0;
    int length_line = 0;
    int seg_id_v = 0;
    vector<Vbox> Vboxes;
    for (int col = 0; col < this->U_map.cols; col++)
    {
        for (int row = 0; row < this->U_map.rows; row++)
        {
           
            if (this->U_map.at<uchar>(row, col) >= u_min)
            {
                // update current line info
                length_line++;
                sum_line += this->U_map.at<uchar>(row, col);
                max_line = (this->U_map.at<uchar>(row, col) > max_line) ? this->U_map.at<uchar>(row, col) : max_line;
            }
           
	    if (this->U_map.at<uchar>(row, col) < u_min || row == this->U_map.rows - 1)
        
            {
                
                row = (row == this->U_map.rows - 1) ? row + 1 : row;
                
                if (length_line > this->min_length_line && sum_line > this->threshold_line * max_line)
                {
                    seg_id_v++;
                    Vboxes.push_back(Vbox(seg_id_v, col, row - length_line, row - 1));
		 
                   
		    for (int c = row - length_line; c < row - 1; c++)
                    {
                        mask[c][col] = seg_id_v;
                    }
                    
                    if (col != 0)
                    {
                        // merge all parents
			for (int c = row - length_line; c < row - 1; c++)
                        {
			    if (mask[c][col - 1] != 0)
                            {
				if (Vboxes[mask[c][col - 1] - 1].toppest_parent_id_v < Vboxes.back().toppest_parent_id_v)
                                {
				    Vboxes.back().toppest_parent_id_v = Vboxes[mask[c][col - 1] - 1].toppest_parent_id_v;
                                }
                                else
                                {
				    int temp = Vboxes[mask[c][col - 1] - 1].toppest_parent_id_v;
                                    for (int b = 0; b < Vboxes.size(); b++)
                                    {
                                        Vboxes[b].toppest_parent_id_v = (Vboxes[b].toppest_parent_id_v == temp) ? Vboxes.back().toppest_parent_id_v : Vboxes[b].toppest_parent_id_v;
                                    }
                                }
                            }
                        }
                    }
                }





                sum_line = 0;
                max_line = 0;
                length_line = 0;
            }
        }
    }

    // group lines into boxes
    this->bounding_box_U.clear();
    // merge boxes with same toppest parent
    for (int b = 0; b < Vboxes.size(); b++)
    {
        if (Vboxes[b].id_v == Vboxes[b].toppest_parent_id_v)
        {
            for (int s = b + 1; s < Vboxes.size(); s++)
            {
                if (Vboxes[s].toppest_parent_id_v == Vboxes[b].id_v)
                {
                    Vboxes[b] = merge_two_Vbox(Vboxes[b], Vboxes[s]);

                }
            }
         
            if (Vboxes[b].bb_v.area() >= 15)
            {
                this->bounding_box_U.push_back(Vboxes[b].bb_v);
            }
        }
    }
}

void Vdetector::detect()
{
    // extract U map from depth
    this->extract_U_map();

    // extract bounding box from U map
    this->extract_bb_v();

    // extract bounding box
    this->extract_bird_view();

    // extract object's height in depth map
    // this->extract_height();
}



void Vdetector::display_depth()
{   
    double min;
    double max;
    int rows = depth.rows;
    int cols = depth.cols;
    Mat depth_clone;
    depth.copyTo(depth_clone);
    int a = 640;
    int b = 480;
    Mat depth_resize;
    Mat depth_resize_clone;
    resize(depth, depth_resize, Size(), 1, this->col_scale);
    depth_resize.copyTo(depth_resize_clone);
    cv::minMaxIdx(depth_resize, &min, &max);
    cv::convertScaleAbs(depth_resize, depth_resize, 255 / max);
    cvtColor(depth_resize, depth_resize, COLOR_GRAY2RGB);

    cv::minMaxIdx(this->depth, &min, &max);
    cv::convertScaleAbs(this->depth, this->depth, 255 / max);
    cvtColor(this->depth, this->depth, COLOR_GRAY2RGB);

    float histSize = this->depth.cols / this->row_downsample;
    float bin_width = ceil((this->max_dist - this->min_dist) / histSize);
    


    for (int b = 0; b < this->bounding_box_U.size(); b++) {

      
	int y=this->bounding_box_U[b].tl().y;
        
	int x_right=depth_resize.cols;
	int x_left=0;
      
	int height= this->bounding_box_U[b].height;
        
	int bin_index_small = this->bounding_box_U[b].tl().x;
        int bin_index_large = this->bounding_box_U[b].br().x - floor((this->bounding_box_U[b].br().x - this->bounding_box_U[b].tl().x) / 2.) ;
        float depth_in_near = (bin_index_small * bin_width + this->min_dist);
        float depth_in_far = (bin_index_large * bin_width + this->min_dist);

        int accumulator = 0;
	for (int j = y; j < y+height/2.0; j++) { // for each coloum
	    for (int i = 0; i < depth_resize_clone.cols - 1; i++) { // for each row
                if (float(depth_resize_clone.at<unsigned short>(j, i)) >= depth_in_near && float(depth_resize_clone.at<unsigned short>(j, i)) <= depth_in_far) {
                    
                    accumulator =0;

                    for (int check = 0; check < 15; check++) { // check some more points in the coloum
                        if (float(depth_resize_clone.at<unsigned short>(j , i+ check + 1)) < depth_in_near || float(depth_resize_clone.at<unsigned short>(j , i+ check + 1)) > depth_in_far) {
                            
                            break;
                        }
                        else{
                            accumulator ++;
                        }
                        if (accumulator == 12) {
                            
                           // if (y_up > j) y_up = j;
			    if (x_right > i) x_right = i;
                            //if (y_down < j) y_down = j;
			    if (x_left < i) x_left = i;
                        }
                        
                    }
                }

            }

        }
        
	int width = -x_right +x_left;
       
	cv::Rect rect(x_right,y,width, height);
        rectangle(depth_resize, rect, cv::Scalar(0, 0, 255), 5, 8, 0);
       
	cv::Rect rect2(x_right, y/ col_scale,width, height/col_scale);
        rectangle(depth, rect2, cv::Scalar(0, 0, 255), 5, 8, 0);

    }
    
    //imshow("Depth", depth_resize);
    //waitKey(1);
    imshow("Depth2", depth);
    waitKey(1);
}

void Vdetector::display_U_map()
{
    // visualize with bounding box
    if (this->show_bounding_box_U)
    {
        this->U_map = this->U_map * 10;
        cvtColor(this->U_map, this->U_map, COLOR_GRAY2RGB);
        for (int b = 0; b < this->bounding_box_U.size(); b++)
        {
            Rect final_bb_v = Rect(this->bounding_box_U[b].tl(), Size(this->bounding_box_U[b].width, 2 * this->bounding_box_U[b].height));
            rectangle(this->U_map, final_bb_v, Scalar(0, 0, 255), 1, 8, 0);
            circle(this->U_map, Point2f(this->bounding_box_U[b].tl().x + 0.5 * this->bounding_box_U[b].width, this->bounding_box_U[b].br().y), 2, Scalar(0, 0, 255), 5, 8, 0);
        }
    }

    imshow("V map", this->U_map);
    waitKey(1);
}


void Vdetector::extract_bird_view()
{
    // extract bounding boxes in bird's view
    uint8_t histSize = this->depth.cols / this->row_downsample;
    uint8_t bin_width = ceil((this->max_dist - this->min_dist) / float(histSize));
    this->bounding_box_B.clear();
    this->bounding_box_B.resize(this->bounding_box_U.size());

    for (int b = 0; b < this->bounding_box_U.size(); b++)
    {
        // U_map bounding box -> Bird's view bounding box conversion
        float bb_v_depth = this->bounding_box_U[b].br().x * bin_width / 10;
        float bb_v_width = bb_v_depth * this->bounding_box_U[b].width / this->fx;
        float bb_v_height = this->bounding_box_U[b].height * bin_width / 10;
        float bb_v_x = bb_v_depth * (this->bounding_box_U[b].tl().y / this->col_scale - this->px) / this->fx;
        float bb_v_y = bb_v_depth - 0.5 * bb_v_height;
        this->bounding_box_B[b] = Rect(bb_v_x, bb_v_y, bb_v_width, bb_v_height);
    }

    // initialize the bird's view
    this->bird_view = Mat::zeros(500, 1000, CV_8UC1);
    cvtColor(this->bird_view, this->bird_view, COLOR_GRAY2RGB);
}


void Vdetector::display_bird_view()
{
    // center point
    Point2f center = Point2f(this->bird_view.cols / 2, this->bird_view.rows);
    Point2f left_end_to_center = Point2f(this->bird_view.rows * (0 - this->px) / this->fx, -this->bird_view.rows);
    Point2f right_end_to_center = Point2f(this->bird_view.rows * (this->depth.cols - this->px) / this->fx, -this->bird_view.rows);

    // draw the two side lines
    line(this->bird_view, center, center + left_end_to_center, Scalar(0, 255, 0), 3, 8, 0);
    line(this->bird_view, center, center + right_end_to_center, Scalar(0, 255, 0), 3, 8, 0);

    for (int b = 0; b < this->bounding_box_U.size(); b++)
    {
        // change coordinates
        Rect final_bb_v = this->bounding_box_B[b];
        final_bb_v.y = center.y - final_bb_v.y - final_bb_v.height;
        final_bb_v.x = final_bb_v.x + center.x;
        // draw center 
        Point2f bb_v_center = Point2f(final_bb_v.x + 0.5 * final_bb_v.width, final_bb_v.y + 0.5 * final_bb_v.height);
        rectangle(this->bird_view, final_bb_v, Scalar(0, 0, 255), 3, 8, 0);
        circle(this->bird_view, bb_v_center, 3, Scalar(0, 0, 255), 5, 8, 0);
    }

    // show
    resize(this->bird_view, this->bird_view, Size(), 0.5, 0.5);
    imshow("Bird's View", this->bird_view);
    waitKey(1);
}

void Vdetector::add_tracking_result()
{
    Point2f center = Point2f(this->bird_view.cols / 2, this->bird_view.rows);
    for (int b = 0; b < this->tracker.now_bb_v.size(); b++)
    {
        // change coordinates
        Point2f estimated_center = Point2f(this->tracker.now_filter[b].output(0), this->tracker.now_filter[b].output(1));
        estimated_center.y = center.y - estimated_center.y;
        estimated_center.x = estimated_center.x + center.x;
        // draw center 
        circle(this->bird_view, estimated_center, 5, Scalar(0, 255, 0), 5, 8, 0);
        // draw bounding box
        Point2f bb_v_size = Point2f(this->tracker.now_filter[b].output(4), this->tracker.now_filter[b].output(5));
        rectangle(this->bird_view, Rect(estimated_center - 0.5 * bb_v_size, estimated_center + 0.5 * bb_v_size), Scalar(0, 255, 0), 3, 8, 0);
        // draw velocity
        Point2f velocity = Point2f(this->tracker.now_filter[b].output(2), this->tracker.now_filter[b].output(3));
        velocity.y = -velocity.y;
        line(this->bird_view, estimated_center, estimated_center + velocity, Scalar(255, 255, 255), 3, 8, 0);
        for (int h = 1; h < this->tracker.now_history[b].size(); h++)
        {
            // trajectory
            Point2f start = this->tracker.now_history[b][h - 1];
            start.y = center.y - start.y;
            start.x = start.x + center.x;
            Point2f end = this->tracker.now_history[b][h];
            end.y = center.y - end.y;
            end.x = end.x + center.x;
            line(this->bird_view, start, end, Scalar(0, 0, 255), 3, 8, 0);
        }
    }
}

void Vdetector::track()
{
    this->tracker.read_bb_v(this->bounding_box_B);
    this->tracker.check_status();
    this->add_tracking_result();
}





































// UVbox

UVbox::UVbox()
{
    this->id = 0;
    this->toppest_parent_id = 0;
    this->bb = Rect(Point2f(0, 0), Point2f(0, 0));
}

UVbox::UVbox(int seg_id, int row, int left, int right)
{
    this->id = seg_id;
    this->toppest_parent_id = seg_id;
    this->bb = Rect(Point2f(left, row), Point2f(right, row));
}

UVbox merge_two_UVbox(UVbox father, UVbox son)
{
    // merge the bounding box
    int top = (father.bb.tl().y < son.bb.tl().y) ? father.bb.tl().y : son.bb.tl().y;
    int left = (father.bb.tl().x < son.bb.tl().x) ? father.bb.tl().x : son.bb.tl().x;
    int bottom = (father.bb.br().y > son.bb.br().y) ? father.bb.br().y : son.bb.br().y;
    int right = (father.bb.br().x > son.bb.br().x) ? father.bb.br().x : son.bb.br().x;
    father.bb = Rect(Point2f(left, top), Point2f(right, bottom));
    return father;
}

// UVtracker

UVtracker::UVtracker()
{
    this->overlap_threshold = 0.51;
}

void UVtracker::read_bb(vector<Rect> now_bb)
{
    // measurement history
    this->pre_history = this->now_history;
    this->now_history.clear();
    this->now_history.resize(now_bb.size());
    // kalman filters
    this->pre_filter = this->now_filter;
    this->now_filter.clear();
    this->now_filter.resize(now_bb.size());
    // bounding box
    this->pre_bb = this->now_bb;
    this->now_bb = now_bb;
}

void UVtracker::check_status()
{
    for (int now_id = 0; now_id < this->now_bb.size(); now_id++)
    {
        bool tracked = false;
        for (int pre_id = 0; pre_id < this->pre_bb.size(); pre_id++)
        {
            Rect overlap = this->now_bb[now_id] & this->pre_bb[pre_id];
            if (min(overlap.area() / float(this->now_bb[now_id].area()), overlap.area() / float(this->now_bb[now_id].area())) >= this->overlap_threshold)
            {
                tracked = true;
                // add current detection to history
                this->now_history[now_id] = this->pre_history[pre_id];
                this->now_history[now_id].push_back(Point2f(this->now_bb[now_id].x + 0.5 * this->now_bb[now_id].width, this->now_bb[now_id].y + 0.5 * this->now_bb[now_id].height));
                // add measurement to previous filter
                this->now_filter[now_id] = this->pre_filter[pre_id];
                MatrixXd z(4, 1); // measurement
                z << this->now_bb[now_id].x + 0.5 * this->now_bb[now_id].width, this->now_bb[now_id].y + 0.5 * this->now_bb[now_id].height, this->now_bb[now_id].width, this->now_bb[now_id].height;
                MatrixXd u(1, 1); // input
                u << 0;
                // run the filter 
                this->now_filter[now_id].estimate(z, u);
                break;
            }
        }
        if (!tracked)
        {
            // add current detection to history
            this->now_history[now_id].push_back(Point2f(this->now_bb[now_id].x + 0.5 * this->now_bb[now_id].width, this->now_bb[now_id].y + 0.5 * this->now_bb[now_id].height));
            // initialize filter
            int f = 30; // Hz
            double ts = 1.0 / f; // s

            // model for center filter
            double e_p = 0.4;
            double e_ps = 0.0;
            double e_m = 0.3;
            double e_ms = 0.99;
            MatrixXd A(6, 6);
            A << 1, 0, ts, 0, 0, 0,
                0, 1, 0, ts, 0, 0,
                0, 0, 1, 0, 0, 0,
                0, 0, 0, 1, 0, 0,
                0, 0, 0, 0, 1, 0,
                0, 0, 0, 0, 0, 1;
            MatrixXd B(6, 1);
            B << 0, 0, 0, 0, 0, 0;
            MatrixXd H(4, 6);
            H << 1, 0, 0, 0, 0, 0,
                0, 1, 0, 0, 0, 0,
                0, 0, 0, 0, 1, 0,
                0, 0, 0, 0, 0, 1;
            MatrixXd P = MatrixXd::Identity(6, 6) * e_m;
            P(4, 4) = e_ms; P(5, 5) = e_ms;
            MatrixXd Q = MatrixXd::Identity(6, 6) * e_p;
            Q(4, 4) = e_ps; Q(5, 5) = e_ps;
            MatrixXd R = MatrixXd::Identity(4, 4) * e_m;

            // filter initialization
            MatrixXd states(6, 1);
            states << this->now_bb[now_id].x + 0.5 * this->now_bb[now_id].width, this->now_bb[now_id].y + 0.5 * this->now_bb[now_id].height, 0, 0, this->now_bb[now_id].width, this->now_bb[now_id].height;
            kalman_filter my_filter;

            this->now_filter[now_id].setup(states,
                A,
                B,
                H,
                P,
                Q,
                R);
        }
    }
}

// UVdetector

UVdetector::UVdetector()
{
    this->row_downsample = 5;
    this->col_scale = 0.5;
    this->min_dist = 10;
    this->max_dist = 2000; //5000
    this->threshold_point = 2;// 2
    this->threshold_line = 6;
    this->min_length_line = 8;
    this->show_bounding_box_U = true;
    this->fx = 383.91;
    this->fy = 383.91;
    this->px = 318.27;
    this->py = 242.18;

    // maybe l515
    // this->fx = 228.7480;
    // this->fy = 229.0078;
    //this->px = 159.3047;
    //this->py = 123.9873;
}

void UVdetector::readdata(Mat depth)
{
    this->depth = depth;
}

void UVdetector::extract_U_map()
{
    // rescale depth map
    Mat depth_rescale;
    resize(this->depth, depth_rescale, Size(), this->col_scale, 1);
    Mat depth_low_res_temp = Mat::zeros(depth_rescale.rows, depth_rescale.cols, CV_8UC1);

    // construct the mask
    Rect mask_depth;
    uint8_t histSize = this->depth.rows / this->row_downsample;
    uint8_t bin_width = ceil((this->max_dist - this->min_dist) / float(histSize));
    // initialize U map
    this->U_map = Mat::zeros(histSize, depth_rescale.cols, CV_8UC1);
    for (int col = 0; col < depth_rescale.cols; col++)
    {
        for (int row = 0; row < depth_rescale.rows; row++)
        {
            if (depth_rescale.at<unsigned short>(row, col) > this->min_dist && depth_rescale.at<unsigned short>(row, col) < this->max_dist)
            {
                uint8_t bin_index = (depth_rescale.at<unsigned short>(row, col) - this->min_dist) / bin_width;
                depth_low_res_temp.at<uchar>(row, col) = bin_index;
                if (this->U_map.at<uchar>(bin_index, col) < 255)
                {
                    this->U_map.at<uchar>(bin_index, col)++;
                }
            }
        }
    }
    this->depth_low_res = depth_low_res_temp;

    // smooth the U map
    GaussianBlur(this->U_map, this->U_map, Size(5, 11), 3, 10);
    cout << "histSize=" << endl;
    cout << histSize << endl;
    // cout<<depth_rescale.cols<<endl;
}

void UVdetector::extract_bb()
{
    // initialize a mask
    vector<vector<int> > mask(this->U_map.rows, vector<int>(this->U_map.cols, 0));
    // initialize parameters
    int u_min = this->threshold_point * this->row_downsample;
    int sum_line = 0;
    int max_line = 0;
    int length_line = 0;
    int seg_id = 0;
    vector<UVbox> UVboxes;
    for (int row = 0; row < this->U_map.rows; row++)
    {
        for (int col = 0; col < this->U_map.cols; col++)
        {
            // is a point of interest
            if (this->U_map.at<uchar>(row, col) >= u_min)
            {
                // update current line info
                length_line++;
                sum_line += this->U_map.at<uchar>(row, col);
                max_line = (this->U_map.at<uchar>(row, col) > max_line) ? this->U_map.at<uchar>(row, col) : max_line;
            }
            // is not or is end of row
            if (this->U_map.at<uchar>(row, col) < u_min || col == this->U_map.cols - 1)
            {
                // is end of the row
                col = (col == this->U_map.cols - 1) ? col + 1 : col;
                // is good line candidate (length and sum)
                if (length_line > this->min_length_line && sum_line > this->threshold_line * max_line)
                {
                    seg_id++;
                    UVboxes.push_back(UVbox(seg_id, row, col - length_line, col - 1));
                    // overwrite the mask with segementation id
                    for (int c = col - length_line; c < col - 1; c++)
                    {
                        mask[row][c] = seg_id;
                    }
                    // when current row is not first row, we need to merge neighbour segementation
                    if (row != 0)
                    {
                        // merge all parents
                        for (int c = col - length_line; c < col - 1; c++)
                        {
                            if (mask[row - 1][c] != 0)
                            {
                                if (UVboxes[mask[row - 1][c] - 1].toppest_parent_id < UVboxes.back().toppest_parent_id)
                                {
                                    UVboxes.back().toppest_parent_id = UVboxes[mask[row - 1][c] - 1].toppest_parent_id;
                                }
                                else
                                {
                                    int temp = UVboxes[mask[row - 1][c] - 1].toppest_parent_id;
                                    for (int b = 0; b < UVboxes.size(); b++)
                                    {
                                        UVboxes[b].toppest_parent_id = (UVboxes[b].toppest_parent_id == temp) ? UVboxes.back().toppest_parent_id : UVboxes[b].toppest_parent_id;
                                    }
                                }
                            }
                        }
                    }
                }
                sum_line = 0;
                max_line = 0;
                length_line = 0;
            }
        }
    }

    // group lines into boxes
    this->bounding_box_U.clear();
    // merge boxes with same toppest parent
    for (int b = 0; b < UVboxes.size(); b++)
    {
        if (UVboxes[b].id == UVboxes[b].toppest_parent_id)
        {
            for (int s = b + 1; s < UVboxes.size(); s++)
            {
                if (UVboxes[s].toppest_parent_id == UVboxes[b].id)
                {
                    UVboxes[b] = merge_two_UVbox(UVboxes[b], UVboxes[s]);

                }
            }
            // check box's size
            if (UVboxes[b].bb.area() >= 25)
            {
                this->bounding_box_U.push_back(UVboxes[b].bb);
            }
        }
    }
}

void UVdetector::detect()
{
    // extract U map from depth
    this->extract_U_map();

    // extract bounding box from U map
    this->extract_bb();

    // extract bounding box
    this->extract_bird_view();

    // extract object's height in depth map
    // this->extract_height();
}

void UVdetector::display_depth()
{   //depth = (depth)/100.;
    double min;
    double max;
    int rows = depth.rows;
    int cols = depth.cols;
    Mat depth_clone;
    // = Mat::create(rows, cols, CV_8UC1);
    depth.copyTo(depth_clone);
    int a = 640;
    int b = 480;
    //
        //Mat depth_rescale;
        //resize(this->depth, depth_rescale, Size(),this->col_scale , 1);
        //Mat depth_low_res_temp = Mat::zeros(depth_rescale.rows, depth_rescale.cols, CV_8UC1);
        //
    Mat depth_resize;
    Mat depth_resize_clone;
    resize(depth, depth_resize, Size(), this->col_scale, 1);
    depth_resize.copyTo(depth_resize_clone);
    cv::minMaxIdx(depth_resize, &min, &max);
    cv::convertScaleAbs(depth_resize, depth_resize, 255 / max);
    cvtColor(depth_resize, depth_resize, COLOR_GRAY2RGB);

    //// cout<< depth_resize<<endl;
    cv::minMaxIdx(this->depth, &min, &max);
    cv::convertScaleAbs(this->depth, this->depth, 255 / max);
    cvtColor(this->depth, this->depth, COLOR_GRAY2RGB);

    float histSize = this->depth.rows / this->row_downsample;
    float bin_width = ceil((this->max_dist - this->min_dist) / histSize);
    
    for (int b = 0; b < this->bounding_box_U.size(); b++) {

        int x = this->bounding_box_U[b].tl().x;
        int y_up = depth_resize.rows;
        int y_down = 0;
        int width = this->bounding_box_U[b].width;
        int bin_index_small = this->bounding_box_U[b].tl().y;
        int bin_index_large = this->bounding_box_U[b].br().y - floor((this->bounding_box_U[b].br().y - this->bounding_box_U[b].tl().y) / 2.) ;
        float depth_in_near = (bin_index_small * bin_width + this->min_dist);
        float depth_in_far = (bin_index_large * bin_width + this->min_dist);

        int accumulator = 0;
        for (int i = x; i < x + width; i++) { // for each coloum
            for (int j = 0; j < depth_resize_clone.rows - 1; j++) { // for each row
                if (float(depth_resize_clone.at<unsigned short>(j, i)) >= depth_in_near && float(depth_resize_clone.at<unsigned short>(j, i)) <= depth_in_far) {
                    
                    accumulator =0;

                    for (int check = 0; check < 15; check++) { // check some more points in the coloum
                        if (float(depth_resize_clone.at<unsigned short>(j + check + 1, i)) < depth_in_near || float(depth_resize_clone.at<unsigned short>(j + check + 1, i)) > depth_in_far) {
                            // bad case
                            break;
                        }
                        else{
                            accumulator ++;
                        }
                        if (accumulator == 12) {
                            // 80% of the consecutive points are in the range 
                            // pass
                            if (y_up > j) y_up = j;
                            if (y_down < j) y_down = j;
                        }
                        //else 
                        // meight be noise
                    }
                }

            }

        }
        int height = y_down - y_up;
        //cv::Rect rect(x, y_up / col_scale, width, height / col_scale);
        //rectangle(this->depth, rect, cv::Scalar(0, 0, 255), 5, 8, 0);
        cv::Rect rect(x, y_up, width, height);
        rectangle(depth_resize, rect, cv::Scalar(0, 255, 255), 5, 8, 0);
        cv::Rect rect2(x / col_scale, y_up, width/col_scale, height);
        rectangle(depth, rect2, cv::Scalar(0, 255, 255), 5, 8, 0);

    }
    //rectangle(this->depth, bb, Scalar(0, 0, 255), 1, 8, 0);
    //circle(this->depth, Point2f(this->bounding_box_U[0].tl().x + 0.5 * this->bounding_box_U[0].width, this->bounding_box_U[0].br().y ), 2, Scalar(0, 0, 255), 5, 8, 0);
    imshow("Depth", this->depth);
    //imshow("Depth", depth_resize);
    waitKey(1);
    //imshow("Depth2", depth);
    //waitKey(1);
}

void UVdetector::display_U_map()
{
    // visualize with bounding box
    if (this->show_bounding_box_U)
    {
        this->U_map = this->U_map * 10;
        cvtColor(this->U_map, this->U_map, COLOR_GRAY2RGB);
        for (int b = 0; b < this->bounding_box_U.size(); b++)
        {
            Rect final_bb = Rect(this->bounding_box_U[b].tl(), Size(this->bounding_box_U[b].width, 2 * this->bounding_box_U[b].height));
            rectangle(this->U_map, final_bb, Scalar(0, 0, 255), 1, 8, 0);
            circle(this->U_map, Point2f(this->bounding_box_U[b].tl().x + 0.5 * this->bounding_box_U[b].width, this->bounding_box_U[b].br().y), 2, Scalar(0, 0, 255), 5, 8, 0);
        }
    }

    imshow("U map", this->U_map);
    waitKey(1);
}

void UVdetector::extract_bird_view()
{
    // extract bounding boxes in bird's view
    uint8_t histSize = this->depth.rows / this->row_downsample;
    uint8_t bin_width = ceil((this->max_dist - this->min_dist) / float(histSize));
    this->bounding_box_B.clear();
    this->bounding_box_B.resize(this->bounding_box_U.size());

    for (int b = 0; b < this->bounding_box_U.size(); b++)
    {
        // U_map bounding box -> Bird's view bounding box conversion
        float bb_depth = this->bounding_box_U[b].br().y * bin_width / 10;
        float bb_width = bb_depth * this->bounding_box_U[b].width / this->fx;
        float bb_height = this->bounding_box_U[b].height * bin_width / 10;
        float bb_x = bb_depth * (this->bounding_box_U[b].tl().x / this->col_scale - this->px) / this->fx;
        float bb_y = bb_depth - 0.5 * bb_height;
        this->bounding_box_B[b] = Rect(bb_x, bb_y, bb_width, bb_height);
    }

    // initialize the bird's view
    this->bird_view = Mat::zeros(500, 1000, CV_8UC1);
    cvtColor(this->bird_view, this->bird_view, COLOR_GRAY2RGB);
}

void UVdetector::display_bird_view()
{
    // center point
    Point2f center = Point2f(this->bird_view.cols / 2, this->bird_view.rows);
    Point2f left_end_to_center = Point2f(this->bird_view.rows * (0 - this->px) / this->fx, -this->bird_view.rows);
    Point2f right_end_to_center = Point2f(this->bird_view.rows * (this->depth.cols - this->px) / this->fx, -this->bird_view.rows);

    // draw the two side lines
    line(this->bird_view, center, center + left_end_to_center, Scalar(0, 255, 0), 3, 8, 0);
    line(this->bird_view, center, center + right_end_to_center, Scalar(0, 255, 0), 3, 8, 0);

    for (int b = 0; b < this->bounding_box_U.size(); b++)
    {
        // change coordinates
        Rect final_bb = this->bounding_box_B[b];
        final_bb.y = center.y - final_bb.y - final_bb.height;
        final_bb.x = final_bb.x + center.x;
        // draw center 
        Point2f bb_center = Point2f(final_bb.x + 0.5 * final_bb.width, final_bb.y + 0.5 * final_bb.height);
        rectangle(this->bird_view, final_bb, Scalar(0, 0, 255), 3, 8, 0);
        circle(this->bird_view, bb_center, 3, Scalar(0, 0, 255), 5, 8, 0);
    }

    // show
    resize(this->bird_view, this->bird_view, Size(), 0.5, 0.5);
    imshow("Bird's View", this->bird_view);
    waitKey(1);
}

void UVdetector::add_tracking_result()
{
    Point2f center = Point2f(this->bird_view.cols / 2, this->bird_view.rows);
    for (int b = 0; b < this->tracker.now_bb.size(); b++)
    {
        // change coordinates
        Point2f estimated_center = Point2f(this->tracker.now_filter[b].output(0), this->tracker.now_filter[b].output(1));
        estimated_center.y = center.y - estimated_center.y;
        estimated_center.x = estimated_center.x + center.x;
        // draw center 
        circle(this->bird_view, estimated_center, 5, Scalar(0, 255, 0), 5, 8, 0);
        // draw bounding box
        Point2f bb_size = Point2f(this->tracker.now_filter[b].output(4), this->tracker.now_filter[b].output(5));
        rectangle(this->bird_view, Rect(estimated_center - 0.5 * bb_size, estimated_center + 0.5 * bb_size), Scalar(0, 255, 0), 3, 8, 0);
        // draw velocity
        Point2f velocity = Point2f(this->tracker.now_filter[b].output(2), this->tracker.now_filter[b].output(3));
        velocity.y = -velocity.y;
        line(this->bird_view, estimated_center, estimated_center + velocity, Scalar(255, 255, 255), 3, 8, 0);
        for (int h = 1; h < this->tracker.now_history[b].size(); h++)
        {
            // trajectory
            Point2f start = this->tracker.now_history[b][h - 1];
            start.y = center.y - start.y;
            start.x = start.x + center.x;
            Point2f end = this->tracker.now_history[b][h];
            end.y = center.y - end.y;
            end.x = end.x + center.x;
            line(this->bird_view, start, end, Scalar(0, 0, 255), 3, 8, 0);
        }
    }
}

void UVdetector::track()
{
    this->tracker.read_bb(this->bounding_box_B);
    this->tracker.check_status();
    this->add_tracking_result();
}
