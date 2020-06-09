/*
class 만들어야하는데 잘 모르겠음...
현재 : 가상의 경유지 구하는 함수까지는 만들었음
while loop 만들어야하는데 헷갈린다.
*/

double line_eqn(double a, double b, double c, double x, double y){
    return a*x + b*y + c;
}
double xy_distance(double a, double b, double c, double x, double y){
    double distance;
    // if distance < 0 : left side, if distance > 0 : right side
    distance = (a*x+b*y+c)/sqrt(pow(a, 2) + pow(b, 2));
    return distance;
}

void path_planning(){
    double a, b, c, x, y;
    double line_eqn = line_eqn(a, b, c, x, y);
    double pole_distance, min_pole_distance, pole_line_distance;
    int i, j, min_pole_num = 0, detour_pole = 0, detour_red = 0;

    double robot_orientation = sth;// Hough Transform 또는 IMU로부터 얻은 robot_orientation TODO!!!!!!!
    
    std::vector<vector<double>> pole_xy{{1,1.5}, {2.5,0.7}, {2.5,2.3}, {4, 1.5}}; // 나중에 내가 짠 코드로 바꿔야할 수도
    std::vector<double> virtual_location; // 여기에 가상의 경유지 저장 
    // Need to receive y_target, x_target, y_robot, x_robot
    a = y_target - y_robot;
    b = x_robot - x_target;
    c = y_target*x_robot + y_robot*x_target;


    //  Need pole's x, y coordinate

    // Align with blue ball -> HOW?
    if(x_target < 0){ // if targeted blue ball is in left side of robot
        while(x_target < 0)
        wheelController.setSpeed(linear_speed, angular_speed); // turn_left
    }
    else if(x_target > 0) { // if targeted blue ball is in right side of robot
        while(x_target > 0)
        wheelController.setSpeed(linear_speed, angular_speed); // turn_right
    }    
    else;

    // checking nearest pole
    for(i = 0; i < pole_xy.size(); i++){
        //pole_distance = sqrt(pow(x_robot-pole_xy[i][0], 2) + pow(y_robot-pole_xy[i][1], 2));
        pole_distance = sqrt(pow(pole_xy[i][0], 2) + pow(pole_xy[i][1], 2)); // if this is using my code
        if(pole_xy[i][2] > 270 || pole_xy[i][2] < 90){
            if(i == 0) min_pole_distance = pole_distance;
            else{
                if(pole_distance < min_pole_distance) {
                    min_pole_distance = pole_distance;
                    min_pole_num = i;
                }
            }
        } 
    }

    // checking nearest red ball that is seen
    for(i=0; i < red_x.size(); i++){
        red_distance = sqrt(pow(red_x[i], 2)+pow(red_y[i], 2));
        if(i == 0) {
            min_red_distance = red_distance;
            min_red_num = 1; // 왜냐면 default는 min_red_num = 0이므로 (시야 내에 피할 빨간공이 없을 때는 min_red_num = 0)
        }
        else{
            if(red_distance < min_red_distance){
                min_red_distance = pole_distance;
                min_red_num = i+1;
            }
        }
    }

    // 직선 경로 내에 부딪힐만한 장애물이 있는지 판단
    // 장애물이 있다면 가상의 경유지 값 구하기
    if(abs(pole_xy[i][0]) < robot_length/2) detour_pole = 1;
    if(abs(red_x) < robot_length/2) detour_red = 1;

    if(min_pole_distance <= min_red_distance){
        if(detour_pole == 1){
            virtual_location = set_virtual_location(a, b, c, 1, 0);
        }
    }
    else{
        if(detour_red == 1){
            virtual_location = set_virtual_location(a, b, c, 0, 1);
        }
        else if(detour_pole == 1){
            virtual_location = set_virtual_location(a, b, c, 1, 0);
        }
    }  

    // 구한 가상의 경유지대로 움직여야함!!!!!!!!!!!!!!!!!!!!! TODO!!!!!!!!!
}




std::vector<double> set_virtual_location_pole(double a, double b, double c, int pole_mode, int red_mode){
    double virtual_x, virtual_y;
    double del_x = abs(0.3*a/sqrt(pow(a, 2)+1));
    double del_y = abs(0.3/sqrt(pow(a, 2)+1));
    std::vector<vector<double>> vir_location;
    if(pole_mode == 1){ // 기둥 피해야함 
        if(a >= 0){
            if(line_eqn(a, b, c, pole_xy[min_pole_num][0], pole_xy[min_pole_num][1]) < 0) {
                virtual_x = pole_xy[min_pole_num][0] + del_x;
                virtual_y = pole_xy[min_pole_num][1] - del_y;
            }
            else{
                virtual_x = pole_xy[min_pole_num][0] - del_x;
                virtual_y = pole_xy[min_pole_num][1] + del_y;
            }
        }
        else{
            if(line_eqn(a, b, c, pole_xy[min_pole_num][0], pole_xy[min_pole_num][1]) < 0) {
                virtual_x = pole_xy[min_pole_num][0] + del_x;
                virtual_y = pole_xy[min_pole_num][1] + del_y;
            }
            else{
                virtual_x = pole_xy[min_pole_num][0] - del_x;
                virtual_y = pole_xy[min_pole_num][1] - del_y;
            }        
        }        
    }
    else{ // 빨간공 피해야함
        if(a >= 0){
            if(red_x[min_red_num] < 0) {
                virtual_x = x_robot + red_x[min_red_num] + del_x;
                virtual_y = y_robot + red_y[min_red_num] - del_y;
            }
            else{
                virtual_x = x_robot + red_x[min_red_num] - del_x;
                virtual_y = y_robot + red_y[min_red_num] + del_y;
            }
        }
        else{
            if(line_eqn(a, b, c, pole_xy[min_pole_num][0], pole_xy[min_pole_num][1]) < 0) {
                virtual_x = x_robot + red_x[min_red_num] + del_x;
                virtual_y = y_robot + red_y[min_red_num] + del_y;
            }
            else{
                virtual_x = x_robot + red_x[min_red_num] - del_x;
                virtual_y = y_robot + red_y[min_red_num] - del_y;
            }        
        }       
    }


    vir_location.push_back(virtual_x);
    vir_location.push_back(virtual_y);

    return vir_location;
}


void detour_red_ball(){
    // 

}

void detour_pole(){

}

void go_to_targeted_blueball(){

}

void go_to_goal(){
    // 초록공이 보일 때까지 반시계 방향 회전 -> 장애물이 주변에 있는 경우는 잠시 보류
    while(green_x.size() == 0) wheelController.setSpeed(0, angular_speed); // 회전
    if(green_x[0] < 0){ // if targeted green ball is in left side of robot
        while(green_x[0] < 0)
        wheelController.setSpeed(0, angular_speed); // turn_left
    }
    else if(green_x[0] > 0) { // if targeted green ball is in right side of robot
        while(green_x[0] > 0)
        wheelController.setSpeed(0, angular_speed); // turn_right
    }    
    else;

    // 초록공까지의 경로에 장애물 있는지 파악
    path_planning();

}
