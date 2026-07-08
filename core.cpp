/*
June 1,2021
Corrected setting errors of LEG, autoH, and autoHs.
Corrected (K0脚振り角度)

*/


#include < ode/ode.h >
#include < drawstuff/drawstuff.h >
#include < stdio.h >
#include < stdlib.h >
#include < windows.h 	>
#include < fstream >
#include < iostream >
#include < string >
#include < sstream >
#include < iomanip >
#include < math.h >
#include < conio.h >  
#include  "biped.h"
#include  "core.h"

#define LEG 180.0	//Update in June 1,2021 :Before revision(#define LEG 190.0)

using namespace std;
core::core(void){
	adjFR	=2.04;
	autoH	=170;	//Update in June 1,2021 :Before revision(autoH=180)
	autoHs	=180;	//Update in June 1,2021 :Before revision(autoHs=190)
	mode		=0;
	walkF	=0;
	pitch	=0;
	roll		=0;
}

core::~core(void){}


// ******************
// *	*  脚位置制御  **
// ******************
void core::footCont(float x,float y,float h,int s){
//x:中点を0とする足前後方向距離（前+）
//y:中点を0とする足左右方向距離（右+）
//h:足首ロール軸から股関節ロール軸までの距離
//s:支持脚(0)遊脚(1)を指定
	float k;

	k = sqrt(x*x+(y*y+h*h));	//A0からK0までの距離
	if(k>LEG)k=LEG;			//計算エラー回避

	x = asin(x/k);			//K0脚振り角度 Update in June 1,2021 :Before revision( x=asin(x/LEG) )

	k = acos(k/LEG);			//K0膝曲げ角度

	fbAV=0;					//UVC評価の為、ジャイロは無効にする
	lrAV=0;
	K0W[s]	= k+x;
	HW[s]	= k*2;
	A0W[s]	= k-x-0.003*fbAV;
	k = atan(y/h);			//K1角度
	K1W[s] = k;
	if(s==0)	A1W[s] = -k-0.002*lrAV;
	else		A1W[s] = -k+0.002*lrAV;
}

// **********************
// *	*  歩行制御メイン  **
// **********************
void core::walk(void){
	short i,j;
	float k;

	switch(mode){

	////////////////////////
	//// 初期姿勢に移行 ////
	////////////////////////
	case 0:
		if(autoHs>autoH)	autoHs-=1;	
		else				mode=10;
		footCont( -adjFR, 0, autoHs,  0 );
		footCont( -adjFR, 0, autoHs,  1 );
		break;

	//////////////////////
	//// アイドル状態 ////
	//////////////////////
	case 10:
		K0W[0]	=  0;
		K0W[1]	=  0;

		//// パラメータ初期化 ////
		dx[0]	=0;
		dx[1]	=0;
		fwr0		=0;
		fwr1		=0;
		fwct		=0;
		dxi		=0;
		dyi		=0;
		dy		=0;
		jikuasi	=0;
		fwctEnd	=48;
		swf		=12;
		fhMax	=20;
		landRate=0.2;
		fh		=0;

		footCont( -adjFR, 0, autoH, 0 );
		footCont( -adjFR, 0, autoH, 1 );

		if(walkF&0x01){
			fw=20;
			mode=20;
		}
		break;


	/////////////////////////////////////////////////////////////////
	//////////////////////// 　歩行制御   ///////////////////////////
	/////////////////////////////////////////////////////////////////

	case 20:
	case 30:





		//###########################################################
		//###################  UVC(上体垂直制御) ####################
		//###########################################################
		if((jikuasi==0 && asiPress_r<-0.1 && asiPress_l>-0.1) ||
			 (jikuasi==1 && asiPress_r>-0.1 && asiPress_l<-0.1)){

			k = 1.5 * 193 * sin(lrRad);	//// 左右方向変位 ////
			if(jikuasi==0)	dyi += k;
			else				dyi -= k;
			if(dyi>0)		dyi=0;
			if(dyi<-30)		dyi=-30;
			k = 1.5 * 130 * sin(fbRad);	//// 前後方向変位 ////
			dxi += k;
		}
		dyi*=0.90;						//減衰
		if(uvcOff==1){
			dxi=0;
			dyi=0;
		}





		//###########################################################
		//########################  基本歩容  #######################
		//###########################################################

		//// 横振り ////
		k=swf*sinf(M_PI*(fwct)/fwctEnd);//sinカーブ
		if(jikuasi==0)	dy=  k;//右振り
		else				dy= -k;//左振り

		//// 軸足側前振り制御 ////
		if(fwct<fwctEnd/2)	dx[jikuasi] =      fwr0*(1-2.0*fwct/fwctEnd  );	//立脚中期まで
		else					dx[jikuasi] = -(fw-dxi)*(  2.0*fwct/fwctEnd-1);	//立脚中期移行UVC適用

		//// 遊脚側前振り制御 ////
		if(mode==20){								//両脚シフト期間
			if( fwct<(landRate*fwctEnd) ){
				dx[jikuasi^1] = fwr1-(fwr0-dx[jikuasi]);
			}
			else{
				fwr1=dx[jikuasi^1];
				mode=30;
			}
		}

		if(mode==30){								//前振出
			k=(
				-cosf(
					M_PI*( fwct-landRate*fwctEnd )/
					( (1-landRate)*fwctEnd )			//前振り頂点までの残りクロック数
				)+1
			)/2;										//0-1の∫的カーブ
			dx[jikuasi^1] = fwr1+k*( fw-dxi-fwr1 );
		}
		if(dx[jikuasi]> 100){							//振り出し幅リミット
			dxi		   -= dx[jikuasi]-100;
			dx[jikuasi] = 100;
		}
		if(dx[jikuasi]<-100){
			dxi		   -= dx[jikuasi]+100;
			dx[jikuasi] =-100;
		}
		if(dx[jikuasi^1]> 100) dx[jikuasi^1] = 100;	//振り出し幅リミット抑制
		if(dx[jikuasi^1]<-100) dx[jikuasi^1] =-100;

		//// 足上制御 ////
		i=landRate*fwctEnd;
		if( fwct>i )	fh = fhMax * sinf( M_PI*(fwct-i)/(fwctEnd-i) );
		else			fh = 0;

		//// 脚制御関数呼び出し ////
		if(jikuasi==0){
			footCont( dx[0]-adjFR	, -dy-dyi+1, autoH,		0 );
			footCont( dx[1]-adjFR	,  dy-dyi+1, autoH-fh,	1 );
		}
		else{
			footCont( dx[0]-adjFR	, -dy-dyi+1, autoH-fh,	0 );
			footCont( dx[1]-adjFR	,  dy-dyi+1, autoH,		1 );
		}





		//###########################################################
		//###########  CPG（歩周期生成、今回は簡易仕様） ############
		//###########################################################
		if(fwct==fwctEnd){	//簡易仕様（固定周期）
			jikuasi^=1;
			fwct=1;
			dxi=0;
			fwr0 = dx[jikuasi];
			fwr1 = dx[jikuasi^1];
			fh=0;
			mode=20;
			if(fw==20){			//歩幅制御			
				landRate=0.1;
				fw=40;
			}
		}
		else  ++fwct;
		break;
	}
}