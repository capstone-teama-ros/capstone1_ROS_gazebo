// Ball Harvesting Zone 들어오자마자 90도 돌면서 찾기
void search_blue_ball1(){
	int i;
	double min_blue_num;
	double min_blue_distance;
	double existing_theta;
  std::vector<double> found_blue;
  
	while(turned_angle != M_PI/2 && found_blue.size() < 3){
	  for(i = 0; i<blue_x.size(); i++){
		  if(i == 0) {
			  min_blue_distance = pow(blue_x[i], 2) + pow(blue_y[i], 2);
			  min_blue_num = 0;
		  }
		  else{
		  	if((pow(blue_x[i], 2) + pow(blue_y[i], 2)) < min_blue_distance){
		  		min_blue_num = i;
		  		min_blue_distance = pow(blue_x[i], 2) + pow(blue_y[i], 2);
	  		}
  		}
    }
 	}
}

// 선택한 파란 공이 화면 중심에 올 때까지 회전
void align_blue_ball(int blue_num){

}

// 회전한 각도 return
double angle_count(){

}
