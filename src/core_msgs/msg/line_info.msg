Header header

#### Line Tracer Mode data ####

# 요철 구간인가? (true/false)
bool is_bump

# 라인트레이싱할 때 쓸 구간 (1/2/3/4/5)
int32 section

# 직사각형의 위치, 크기 정보: (x, y, w, h) (단위: 픽셀)
int32 x
int32 y
int32 w
int32 h

# 이미지의 전체 크기 (직사각형의 상대적 위치를 파악하기 위해 필요합니다)
int32 image_width
int32 image_height
