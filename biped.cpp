/*
June 1,2021
Delete variable K2Ｗ[2]
Fixed restart ()

ODEによるUVC(上体垂直制御)の検証
UVCの応用、登坂実験 2021 4/29
*/

#include < ode/ode.h >
#include < drawstuff/drawstuff.h >
#include < stdio.h >
#include < stdlib.h >
#include < windows.h >
#include < fstream >
#include < math.h >
#include < conio.h >
#include  "biped.h"
#include  "core.h"

//関数定義
static	void command		(int cmd);
static	void nearCallback	(void *data, dGeomID o1, dGeomID o2);
static	void simLoop		(int pause);
static	void setJoint		(jointStr *j, char k, bodyStr *b1, bodyStr *b2, char a, double x, double y, double z);
static	void setBody		(bodyStr  *b, char k, char	 c,	 double l,	 double w, double h, double r, double x, double y, double z,  int ge,   double ma);
static	void createBody		();
void		 destroyBot		();
void		 restart		();
static	void start			();

// select correct drawing functions
#ifdef  dDOUBLE			//単精度と倍精度の両方に対応するためのおまじない
#define dsDrawBox		dsDrawBoxD
#define dsDrawSphere	dsDrawSphereD
#define dsDrawCylinder	dsDrawCylinderD
#define dsDrawCapsule	dsDrawCapsuleD
#endif

//間接角度
double K0W[2]={0,0};	//股関節前後方向書込用
double K1W[2]={0,0};	//股関節横方向書込用
//double K2W[2]={0,0};	//股関節横方向書込用
double HW [2]={0,0};	//膝関節書込用
double A0W[2]={0,0};	//足首上下方向書込用
double A1W[2]={0,0};	//足首横方向書込用
double U0W[2]={0,0};	//肩前後方向書込用
double U1W[2]={0,0};	//肩横後方向書込用
double U2W[2]={0,0};	//肩ヨー向書込用

//センサ関連
double fbRad=0;			//頭前後角度
double lrRad=0;			//頭左右角度
double fbAV=0;			//頭前後角速度
double lrAV=0;			//頭左右角速度
double asiPress_r=0;	//右足裏圧力
double asiPress_l=0;	//左足裏圧力

//各種変数定義
double softERP;			//柔らかさ、沈み込み
double softCFM;			//柔らかさ、弾力
double bounce;			//反発係数
double bounce_vel;		//反発最低速度

static dWorldID world;				//動力学計算用ワールド
static dSpaceID space;				//衝突検出用スペース
static dJointGroupID contactgroup;	//コンタクトグループ
static dGeomID ground;				//地面

dMatrix3 R;
const double* Rot;		// 回転行列取得
int		uvcOff=0;		//UVC起動フラグ
unsigned char walkF=0;	//歩行フラグ	（b0:歩行  b1:未  b2:未）
int		bodyCount;		//ボディ配列カウント値
int		jointCount;		//ジョイント配列カウント値
static struct dJointFeedback feedback[50];	//ジョイントフィードバック構造体
double	frRatI;			//上体角補正用積分係数
double	frRatA;			//上体角オフセット値
double	fhRat;			//足上げ高さ補正値
double	fwMax;			//歩幅最大値


//###############  各種構造体　###############
bodyStr *body[50];	//bodyStrアドレス格納配列
bodyStr solep_r;	//足裏圧力センサ
bodyStr solep_l;	
bodyStr sole_r;		//足裏
bodyStr sole_l;	
bodyStr A1_r;		//足首ロール	
bodyStr A1_l;	
bodyStr A0_r;		//足首ピッチ
bodyStr A0_l;
bodyStr S_r;		//脛
bodyStr S_l;
bodyStr H_r;		//膝
bodyStr H_l;
bodyStr M_r;		//腿
bodyStr M_l;
bodyStr K0_r;		//股関節ピッチ
bodyStr K0_l;
bodyStr K1_r;		//股関節ロール
bodyStr K1_l;
bodyStr DOU;		//胴
bodyStr HEADT;		//頭
bodyStr DAI;		//台
bodyStr DAI2;		//台
bodyStr base;		//遮断機柱
bodyStr pole;		//遮断機棒

jointStr *joint[50];//jointStrアドレス格納配列
jointStr soleJ_r;	//足裏センサ
jointStr soleJ_l;
jointStr A1J_r;		//足首ロール
jointStr A1J_l;
jointStr A0J_r;		//足首ピッチ
jointStr A0J_l;
jointStr SJ_r;		//脛固定
jointStr SJ_l;
jointStr HJ_r;		//膝
jointStr HJ_l;
jointStr MJ_r;		//腿結合
jointStr MJ_l;
jointStr M2J_r;		//腿結合2
jointStr M2J_l;
jointStr K0J_r;		//股関節ピッチ
jointStr K0J_l;
jointStr K1J_r;		//股関節ロール
jointStr K1J_l;
jointStr K2J_r;		//股関節ヨー
jointStr K2J_l;
jointStr HEADJ;		//頭固定
jointStr DAIJ;		//台の固定
jointStr DAI2J;		//台2の固定
jointStr baseJ;		//柱の固定
jointStr poleJ;		//ポールのジョイント

//###############  クラスの実体化　###############
core co;			//歩行制御クラスの実体化


//--------------------------------- command ----------------------------------------
static void command (int cmd){
	static int mag = 30;

	switch (cmd) {
		//// 外力印加 ////
		case 'j':case 'J':
			printf("Ｆ←\n");
			dBodyAddForce(DOU.b, -20,0,0);
			break;
		case 'k':case 'K':
			printf("Ｆ→\n");
			dBodyAddForce(DOU.b,  20,0,0);
			break;
		case 'p':case 'P':
			printf("Ｆ↑\n");
			dBodyAddForce(DOU.b, 0,20,0);
			break;
		case 'l':case 'L':
			printf("Ｆ↓\n");
			dBodyAddForce(DOU.b,  0,-20,0);
			break;

		//// 操作 ////
		case 'w':				//歩行開始
			printf("歩行開始\n");
			walkF=0x01;
			poleJ.t=5;
			break;

		case '1':				//外力実験
			printf("外力実験\n");
			restart ();
			fwMax=30;
			frRatA=0;
			frRatI=0;
			fhRat=0;
			setBody  (&base,			'y','d',	260,	 0,	  0,	24,		0,	180,	110,		0,	0.01);	//遮断機棒
			setBody  (&pole,			'y','y',	320,	 0,	  0,	8,		0,	300,	210,		1,	0.0001);	//ボール
			dRFromAxisAndAngle(R, 1, 0, 0, -M_PI_2);//回転
			dBodySetRotation(pole.b, R);
			setJoint(&baseJ,			'g',	&base,	&base,	'x',		0,	180,	0);		//遮断機柱固定用
			setJoint(&poleJ,			'h',	&pole,	&base,	'z',		0,	180,	210);	//遮断機棒ヒンジ
			poleJ.tm=1;
			poleJ.tk=0.25;
			break;

		case '2':				//前進実験
			printf("前進実験\n");
			restart ();
			fwMax=30;
			frRatA=0.0015;
			frRatI=0;
			fhRat=0;
			goto saka;
			
		case '3':				//登坂実験
			printf("登坂実験\n");
			restart ();
			fwMax=30;
			frRatA=0.0015;
			frRatI=0.2;
			fhRat=0;
			goto saka;

		case '4':				//段差実験
			printf("段差実験\n");
			restart ();
			fwMax=30;
			frRatA=0.0015;
			frRatI=0.2;
			fhRat=7;
			setBody  (&DAI2,		'b','g',500,300,	 20,0,		0,	0,	-10,		 1,	100);	//水平台
			setJoint(&DAI2J,		'g',	&DAI2,&DAI2,	'x',	50,	0,	0);						//台固定用
			break;

		case '5':				//最終実験
			printf("最終実験\n");
			restart ();
			fwMax=30;
			frRatI=0.2;
			frRatA=0.0015;
			fhRat=7;
saka:
			setBody  (&DAI,			'b','g',680,300,	 100,   0,	 480,	0,   -70,	1,	100);		//傾斜台
			dRFromAxisAndAngle(R, 0, 1, 0, -0.05);//回転
			dBodySetRotation(DAI.b, R);
			setJoint(&DAIJ,			'g',	&DAI,		&DAI,	'x',	  50,	0,	   0);				//台固定用
			break;

		case 'r':case 'R':		//初期化
			printf("初期化\n");
			restart ();
			break;
		case 'q':case 'Q':		//終了
			printf("終了\n");
			exit(0);
			break;
		case 'u':case 'U':		//UVC ON/OFF
			printf("UVC ON/OFF\n");
			if(uvcOff==0){
				uvcOff=1;
			}
			else{
				uvcOff=0;
			}
			break;
	}
}


//----------------------------------- nearCallback --------------------------------------
static void nearCallback (void *data, dGeomID o1, dGeomID o2){
	int i,n;
	const int N = 10;
	dContact contact[N];
	dBodyID b1 = dGeomGetBody(o1);
	dBodyID b2 = dGeomGetBody(o2);

	if (b1 && b2 && dAreConnectedExcluding(b1,b2,dJointTypeContact)) return;
	n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
	if (n > 0) {
		for (i=0; i<n; i++) {
			contact[i].surface.mode		= dContactBounce | dContactSoftERP | dContactSoftCFM;
			contact[i].surface.soft_cfm	= 0.00005;												//柔らかさ、弾力
			contact[i].surface.soft_erp	= 0.1;													//柔らかさ、沈み込み
			if(pole.b>0 &&(b1==pole.b||b2==pole.b))	contact[i].surface.mu		= 0.2;			//ポールの摩擦
			else									contact[i].surface.mu		= 5;			//地面間摩擦
			contact[i].surface.bounce		= 0;												// bouncing the objects
			contact[i].surface.bounce_vel	= 0;												// bouncing velocity
			dJointID c = dJointCreateContact (world,contactgroup,&contact[i]);					//ジョイント生成
			dJointAttach (c,dGeomGetBody(contact[i].geom.g1),dGeomGetBody(contact[i].geom.g2));	//ジョイントを結合する
	    }
	}
}


//--------------------------------- control ----------------------------------------
// hinge control 
static void control(){
	double kp = 100.0;
	double k;
	int i;
	for (i=0;i<jointCount;i++){
		switch (joint[i]->k) {
			case 'h':
				k = kp * (joint[i]->t - dJointGetHingeAngle(joint[i]->j));
				if(abs(k) > joint[i]->tm){
					if(k > 0){k = joint[i]->tm;}
					else{k = -joint[i]->tm;}
				}
				dJointSetHingeParam(joint[i]->j, dParamVel, k );
				dJointSetHingeParam(joint[i]->j, dParamFMax, joint[i]->tk); 
				break;
			case 'd':
				k = joint[i]->t - dJointGetSliderPosition(joint[i]->j);
				dJointSetSliderParam(joint[i]->j, dParamVel,  k * 100);
				dJointSetSliderParam(joint[i]->j, dParamFMax, 300); 
				break;
		}
	}
}


//--------------------------------- simLoop ----------------------------------------
//	simulation loop
static void simLoop (int pause){
	int i;
	char a;
	static int mag = 3;

	double sides[3];
	dJointFeedback *fb;
	dVector3 headVel1;
	dVector3 headVel2;

//	Sleep(1);			//描画速度の調整
	if(kbhit()){
		a=getchar();	//キー読込
		command (a);
	}

	if (!pause) {
		//******** この３行は最初に置くべし ********
		dSpaceCollide (space,0,&nearCallback);	//衝突しそうなジオメトリのペア集団を探す
		dWorldStep (world,0.01);				//シミュレーションを１ステップ指定時間進める
		dJointGroupEmpty (contactgroup);		//ジョイントグループを空にする

		//******** 足裏圧力検出 ********
		fb = dJointGetFeedback(soleJ_r.j);
		asiPress_r = fb->f1[2];				//右足(足首Ｚ)圧力
		fb = dJointGetFeedback(soleJ_l.j);
		asiPress_l = fb->f1[2];				//左足(足首Ｚ)圧力

		//******** 頭前後左右角度検出 ********
		Rot = dBodyGetRotation(HEADT.b);		//回転行列取得
		fbRad = asin(Rot[8]);					//頭前後角度(後ろに仰向きが正)
		lrRad = asin(Rot[9]);					//頭左右角度（右傾きが正）

		//******** 頭前後左右角速度検出 ********
		Rot = dBodyGetAngularVel(HEADT.b);	//回転行列取得
		fbAV = Rot[1];						//頭前後角度(後ろに仰向きが負)
		lrAV = Rot[0];						//頭左右角度（右傾きが正）

		K0J_r.t	=K0W[0];			//股関節前後方向書込用
		K1J_r.t	=K1W[0];			//股関節横方向書込用
		HJ_r.t	=HW [0];			//膝関節書込用
		A0J_r.t	=A0W[0];			//足首上下方向書込用
		A1J_r.t	=A1W[0];			//足首横方向書込用

		K0J_l.t	=K0W[1];			//股関節前後方向書込用
		K1J_l.t	=K1W[1];			//股関節横方向書込用
		HJ_l.t	=HW [1];			//膝関節書込用
		A0J_l.t	=A0W[1];			//足首上下方向書込用
		A1J_l.t	=A1W[1];			//足首横方向書込用

		co.walk();				//歩行制御
		control();				//モータ駆動
	}

	for (i=0;i<bodyCount;i++){
		switch (body[i]->c) {
			case 'g':
				dsSetColor (0,1,0);
				break;
			case 'r':
				dsSetColor (1,0,0);
				break;
			case 'b':
				if(uvcOff==0)	dsSetColor(0.3 ,0.3, 2.0);			//青
				else			dsSetColor(2.0, 0.3, 0.3);			//赤
				break;
			case 'y':
	 			dsSetColor (1,1,0);
				break;
			case 'w':
	 			dsSetColor (1,1,1);
				break;
			case 'd':
	 			dsSetColor (0.8,0.8,0.8);
				break;
			default:
				break;
		}
		switch (body[i]->k) {
			case 'b':
				sides[0] = body[i]->l; sides[1] = body[i]->w; sides[2] = body[i]->h;
				dsDrawBox (dBodyGetPosition(body[i]->b),dBodyGetRotation(body[i]->b),sides);						//箱形表示
				break;
			case 's':
	 			dsDrawSphere (dBodyGetPosition(body[i]->b),dBodyGetRotation(body[i]->b),body[i]->r);				//球形表示
				break;
			case 'c':
				dsDrawCapsule (dBodyGetPosition(body[i]->b),dBodyGetRotation(body[i]->b),body[i]->l,body[i]->r);	//カプセル形表示
				break;
			case 'y':
				dsDrawCylinder (dBodyGetPosition(body[i]->b),dBodyGetRotation(body[i]->b),body[i]->l,body[i]->r);	//円柱形表示
				break;
			default:
				break;
		}
	}
}


//----------------------------------- setBody --------------------------------------
//	配列にボディ情報を設定する

static void setBody (bodyStr *b, char k,    char c, double l, double w, double h, double r, double x, double y, double z, int ge, double ma){
//引数：　                    ボディの種類     色　    長さ　     幅　      高さ     半径　  前後位置   左右位置　上下位置  ジオメト　重量

	dMass m;

	z += 20;

	//スケール調整
	l/=1000;
	w/=1000;
	h/=1000;
	r/=1000;
	x/=1000;
	y/=1000;
	z/=1000;

	//構造体に記録する
	b-> k = k;			//ボディの種類を記録
	b-> c = c;			//ボディの色の種類を記録
	b-> l = l;			//ボディの長さを記録
	b-> w = w;			//ボディの幅さを記録
	b-> h = h;			//ボディの高さを記録
	b-> r = r;			//ボディの半径を記録
	b-> ge = ge;			//ジオメトリ設定 有/無
	b-> e = 1;			//ボディ有効設定

	x += 2;				//2m手前に置いて地面障害物を避ける

	body[bodyCount] = b;	//構造体のアドレスを格納する
	++bodyCount;			//ボディ数カウントアップ

	//ボディとジオメトリの生成と、質量、位置、関連性を設定
	switch (b->k) {
		case 'b':	//箱型
			b->b = dBodyCreate (world);						// 物体の存在を生成(ボディID設定)
			dMassSetZero(&m);								// 質量パラメータ初期化
			dMassSetBoxTotal (&m,ma,b->l,b->w,b->h);		// 物体の重量設定
			dBodySetMass (b->b,&m);							// 物体の重量分布設定
			dBodySetPosition (b->b,x,y,(z));				// 物体の初期位置
			if(ge > 0){
				b->g = dCreateBox (space,b->l,b->w,b->h);	// 物体の幾何情報を生成（ジオメトリID設定）
				dGeomSetBody (b->g,b->b);					// 物体の『存在』と『幾何情報』の一致
			}
			break;
		case 's':	//球形
			b->b = dBodyCreate (world);						// 物体の存在を生成(ボディID設定)
			dMassSetZero(&m);								// 質量パラメータ初期化
			dMassSetSphereTotal (&m,ma,b->r);				// 物体の重量設定
			dBodySetMass (b->b,&m);							// 物体の重量分布設定
			dBodySetPosition (b->b,x,y,z);					// 物体の初期位置
			if(ge > 0){
				b->g = dCreateSphere (space,b->r);			// 物体の幾何情報を生成（ジオメトリID設定）
				dGeomSetBody (b->g,b->b);					// 物体の『存在』と『幾何情報』の一致
			}
			break;
		case 'c':	//カプセル形
			b->b = dBodyCreate (world);						// 物体の存在を生成(ボディID設定)
			dMassSetZero(&m);								// 質量パラメータ初期化
			dMassSetCapsuleTotal(&m,ma,3,b->r,b->l);		// 物体の重量設定
			dBodySetMass (b->b,&m);							// 物体の重量分布設定
			dBodySetPosition (b->b,x,y,(b->l/2+z));			// 物体の初期位置
			if(ge > 0){
				b->g = dCreateCapsule (space,b->r,b->l);	// 物体の幾何情報を生成（ジオメトリID設定）
				dGeomSetBody (b->g,b->b);					// 物体の『存在』と『幾何情報』の一致
			}
			break;
		case 'y':	//円柱形
			b->b = dBodyCreate (world);						// 物体の存在を生成(ボディID設定)
			dMassSetZero(&m);								// 質量パラメータ初期化
			dMassSetCylinderTotal(&m,ma,3,b->r,b->l);		// 物体の重量設定
			dBodySetMass (b->b,&m);							// 物体の重量分布設定
			dBodySetPosition (b->b,x,y,(z));				// 物体の初期位置
			if(ge > 0){
				b->g = dCreateCylinder (space,b->r,b->l);	// 物体の幾何情報を生成（ジオメトリID設定）
				dGeomSetBody (b->g,b->b);					// 物体の『存在』と『幾何情報』の一致
			}
			break;
		default:
			break;
	}
}


//---------------------------------- setJoint ---------------------------------------
//	ジョイントを生成する

static void setJoint (jointStr *j, char k, bodyStr *b1, bodyStr *b2, char a, double x, double y, double z){
//引数：　            対象Joint　Joint種類   Body番号1  　Body番号2　 設定軸  前後位置  左右位置　上下位置

	z+= 20;

	x/=1000;
	y/=1000;
	z/=1000;

	//構造体に記録する
	j -> k	= k;			//種類を記録
	j -> x	= x;			//X座標を記録
	j -> y	= y;			//X座標を記録
	j -> z	= z;			//X座標を記録
	j -> c	= 0;			//汎用カウンタ
	j -> t	= 0;			//間接角度
	j -> t2	= 0;			//間接角度2
	j -> mode = 0;			//駆動モード
	j -> pn	= 0;			//足圧力カウンタ
	j -> tm	= 44.06;		//8.06最大角速度
	j -> tm2	= 8.06;		//8.06最大角速度
	j -> tk	= 2.45;		//2.45トルク 25kgfcm   (25/100)/9.8=2.45
	j -> tk2	= 2.45;		//トルク2

	x += 2;					//2m手前に置いて地面障害物を避ける
	joint[jointCount] = j;	//配列のアドレスを格納する
	++jointCount;			//配列カウンタ、カウントアップ

	switch (k) {
		case 'h':	//ヒンジジョイント
			j -> j = dJointCreateHinge(world, 0);			// ヒンジジョイントの生成と記録
			dJointAttach(j -> j, b1->b, b2->b);				// ヒンジジョイントの取付
			dJointSetHingeAnchor(j -> j, x, y, z);			// 中心点の設定
			switch (a) {		//軸を設定
				case 'x': dJointSetHingeAxis(j -> j, 1, 0, 0); break;	// X軸の設定
				case 'z': dJointSetHingeAxis(j -> j, 0, 0, 1); break;	// Z軸の設定
				default : dJointSetHingeAxis(j -> j, 0, 1, 0); break;	// Y軸の設定
			}
			break;
		case 'd':	//スライダージョイント（ダンパー）
			j -> j = dJointCreateSlider(world, 0);		// スライダージョイントの生成と記録
			dJointAttach(j -> j, b1->b, b2->b);			// スライダージョイントの取付
			dJointSetSliderAxis(j -> j, 0, 0, 1);		// 中心点の設定
			break;
		case 'f':	//固定ジョイント
			j -> j = dJointCreateFixed(world, 0);		// 固定ジョイントの生成と記録
			dJointAttach(j -> j, b1->b, b2->b);			// 固定ジョイントの取付
			dJointSetFixed(j -> j);						// 固定ジョイントにの設定
			break;
		case 'g':	//環境固定ジョイント
			j -> j = dJointCreateFixed(world, 0);			// 固定ジョイントの生成と記録
			dJointAttach(j -> j, b1->b, 0);				// 固定ジョイントの取付（環境固定）
			dJointSetFixed(j -> j);						// 固定ジョイントにの設定
			break;
		default:
			break;
	}
}


//---------------------------------- createBody ---------------------------------------
//	各部のパーツサイズ等を指定してロボットを組み立てる
static void createBody (){
	double	fw	= 21;		//脚の間隔（中心からの距離）
	double	fmax= 1000;		//駆動トルク標準値
	double	kmax= 1000;		//最大角速度

	softERP		= 0.2;		//弾力
	softCFM		= 0.0001;	//沈み込み
	bounce		= 0.01;		//反発係数
	bounce_vel	= 0.02;		//反発最低速度
	bodyCount	= 0;			//ボディ配列カウント値
	jointCount	= 0;			//ジョイント配列カウント値

//	####################
//	#### ボディ生成 ####
//	####################
//						    種類 色		L　 W	H   R		X	Y	Z	ジオメト 重量

	setBody  (&HEADT,		'c','w',	15,	0,	0,	21,		0,	0,	340,	0,	0.16);	//頭
	setBody  (&DOU,			'b','b',	40, 84, 130,0,		0,	0,	260,	1,	1.24);	//胴
	setBody  (&K0_r,		'y','d',	34,	0,	0,	12,		0,	-fw,195,	0,	0.05);	//股関節ピッチ
	dRFromAxisAndAngle(R, 1, 0, 0, -M_PI_2);//回転
	dBodySetRotation(K0_r.b, R);
	setBody  (&K0_l,		'y','d',	34,	0,	0,	12,		0,	fw,	195,	0,	0.05);
	dRFromAxisAndAngle(R, 1, 0, 0, -M_PI_2);//回転
	dBodySetRotation(K0_l.b, R);
	setBody  (&K1_r,		'y','w',	34,	0,	0,	12,		0,	-fw,195,	0,	0.05);	//股関節ロール
	dRFromAxisAndAngle(R, 0, 1, 0, -M_PI_2);//回転
	dBodySetRotation(K1_r.b, R);
	setBody  (&K1_l,		'y','w',	34,	0,	0,	12,		0,	fw,	195,	0,	0.05);
	dRFromAxisAndAngle(R, 0, 1, 0, -M_PI_2);//回転
	dBodySetRotation(K1_l.b, R);
	setBody  (&M_r,			'b','d',	20,	26,	90,	0,		0,	-fw,150,	0,	0.08);	//腿
	setBody  (&M_l,			'b','d',	20,	26,	90, 0,		0,	fw,	150,	0,	0.08);
	setBody  (&H_r,			'y','d',	34,	0,	0,  12,		0,	-fw,105,	0,	0.03);	//膝
	dRFromAxisAndAngle(R, 1, 0, 0, -M_PI_2);//回転
	dBodySetRotation(H_r.b, R);
	setBody  (&H_l,			'y','d',	34,	0,	0,	12,		0,	fw,	105,	0,	0.03);
	dRFromAxisAndAngle(R, 1, 0, 0, -M_PI_2);//回転
	dBodySetRotation(H_l.b, R);
	setBody  (&S_r,			'b','d',	20,	26, 90,	0,		0,	-fw,60,		1,	0.04);	//脛
	setBody  (&S_l,			'b','d',	20,	26, 90,	0,		0,	fw,	60,		1,	0.04);
	setBody  (&A0_r,		'y','d',	34,	 0, 0,	12,		0,	-fw,15,		0,	0.02);	//足首ピッチ
	dRFromAxisAndAngle(R, 1, 0, 0, -M_PI_2);//回転
	dBodySetRotation(A0_r.b, R);
	setBody  (&A0_l,		'y','d',	34,	 0,	0,	12,		0,	fw,	15,		0,	0.02);
	dRFromAxisAndAngle(R, 1, 0, 0, -M_PI_2);//回転
	dBodySetRotation(A0_l.b, R);
	setBody  (&A1_r,		'y','w',	34,	 0,	0,	12,		0,	-fw,15,		0,	0.02);	//足首ロール
	dRFromAxisAndAngle(R, 0, 1, 0, -M_PI_2);//回転
	dBodySetRotation(A1_r.b, R);
	setBody  (&A1_l,		'y','w',	34,	 0,	0,	12,		0,	fw,	15,		0,	0.02);
	dRFromAxisAndAngle(R, 0, 1, 0, -M_PI_2);//回転
	dBodySetRotation(A1_l.b, R);
	setBody  (&sole_r,		'b','w',	55,	 40,2,	0,		0,	-fw,6.0,	0,	0.01);	//足平
	setBody  (&sole_l,		'b','w',	55,	 40,2,	0,		0,  fw,	6.0,	0,	0.01);
	setBody  (&solep_r,		'b','r',	55,	 40,6,	0,		0,	-fw,3.0,	1,	0.01);	//ソールセンサ
	setBody  (&solep_l,		'b','r',	55,	 40,6,	0,		0,  fw,	3.0,	1,	0.01);
	pole.b=0;//摩擦係数設定条件

//	######################
//	####ジョイント生成####
//	######################
//							種類		B番号1		B番号2	軸			X		Y		Z

	setJoint(&HEADJ,		'f',	&HEADT,		&DOU,	'z',		0,		0,		360);	//頭固定用
	setJoint(&K0J_r,		'h',	&K1_r,		&K0_r,	'y',		0,		-fw,	195);	//股関節ピッチ
	setJoint(&K0J_l,		'h',	&K1_l,		&K0_l,	'y',		0,		fw,		195);
	setJoint(&K1J_r,		'h',	&DOU,		&K1_r,	'x',		0,		-fw+11,	195);	//股関節ロール
	setJoint(&K1J_l,		'h',	&K1_l,		&DOU,	'x',		0,		fw-11,	195);
	setJoint(&MJ_r,			'f',	&M_r,		&K0_r,	'y',		0,		-fw,	128);	//腿固定用
	setJoint(&MJ_l,			'f',	&M_l,		&K0_l,	'y',		0,		fw,		128);
	setJoint(&M2J_r,		'f',	&H_r,		&M_r,	'y',		0,		-fw,	128);	//腿固定用
	setJoint(&M2J_l,		'f',	&H_l,		&M_l,	'y',		0,		fw,		128);
	setJoint(&HJ_r,			'h',	&S_r,		&H_r,	'y',		0,		-fw,	105);	//膝関節
	setJoint(&HJ_l,			'h',	&S_l,		&H_l,	'y',		0,		fw,		105);
	setJoint(&SJ_r,			'f',	&S_r,		&A0_r,	'y',		0,		-fw,	60);	//脛固定用
	setJoint(&SJ_l,			'f',	&S_l,		&A0_l,	'y',		0,		fw,		60);
	setJoint(&A0J_r,		'h',	&A0_r,		&A1_r,	'y',		0,		-fw,	15);	//足首ピッチ
	setJoint(&A0J_l,		'h',	&A0_l,		&A1_l,	'y',		0,		fw,		15);
	setJoint(&A1J_r,		'h',	&A1_r,		&sole_r,'x',		0,		-fw+11,	15);	//足首ロール
	setJoint(&A1J_l,		'h',	&sole_l,	&A1_l,	'x',		0,		fw-11,	15);
	setJoint(&soleJ_r,		'd',	&solep_r,	&sole_r,'x',		0,		-fw,	6);		//ソール圧力センサ
	setJoint(&soleJ_l,		'd',	&solep_l,	&sole_l,'x',		0,		fw,		6);

	dJointSetFeedback(soleJ_r.j,			&feedback[0]);
	dJointSetFeedback(soleJ_l.j,			&feedback[1]);
}


//--------------------------------- destroy ----------------------------------------
//	ロボットの破壊
void destroyBot (){
	int i;
	for (i=0;i<jointCount;i++){
		if(joint[i]->e > 0){dJointDestroy (joint[i]->j);}	//ジョイント破壊
	}
	for (i=0;i<bodyCount;i++){
		if(body[i]->e > 0){dBodyDestroy (body[i]->b);}		//ボディ有効なら破壊
		if(body[i]->ge > 0){dGeomDestroy (body[i]->g);}		//ジオメトリ設定されてたら破壊
	}
	dJointGroupDestroy (contactgroup);
}


//--------------------------------- restart ----------------------------------------
//	リスタート
void restart (){
	destroyBot ();
	contactgroup = dJointGroupCreate (0);	//接触点のグループを格納する入れ物
	createBody();						//ロボット生成
	dWorldSetGravity (world, 0, 0, -9.8);	//重力設定
	co.mode=0;
	co.autoHs=180;
	walkF=0;
	uvcOff=0;
	frRatI=0;
	frRatA=0;
	fhRat=0;
	fwMax=0;
 
	K0W[0]=0;			//股関節前後方向
	K1W[0]=0;			//股関節横方向
	HW [0]=0;			//膝関節
	A0W[0]=0;			//足首上下方向
	A1W[0]=0;			//足首横方向
	K0W[1]=0;			//股関節前後方向
	K1W[1]=0;			//股関節横方向
	HW [1]=0;			//膝関節
	A0W[1]=0;			//足首上下方向
	A1W[1]=0;			//足首横方向
}


//--------------------------------- start ----------------------------------------
//	start simulation - set viewpoint
static void start(){
	static float xyz[3] = {2.3, -0.3, 0.18};	//視点の位置
	static float hpr[3] = {135,0,0};	//視線の方向
	dsSetViewpoint (xyz,hpr);//カメラの設定
}


//------------------------------------ main -------------------------------------
int main (int argc, char **argv){
	dMass m;
	dInitODE(); // ODEの初期化

	// setup pointers to drawstuff callback functions
	dsFunctions fn;
	fn.version = DS_VERSION;
	fn.start = &start;
	fn.step = &simLoop;
//	fn.command = &command;　//Windows10から利用できなくなった
	fn.stop = 0;
	fn.path_to_textures = "c:/ode-0.13/drawstuff/textures";
	if(argc==2)fn.path_to_textures = argv[1];

	//   create world
	world = dWorldCreate();					//シミュレーションワールド生成
	space = dHashSpaceCreate (0);			//衝突検出用スペース生成
	contactgroup = dJointGroupCreate (0);	//接触点のグループを格納する入れ物
	dWorldSetGravity (world, 0, 0, -9.8);	//重力設定
	ground = dCreatePlane (space,0,0,1,0);	//平面のジオメトリ作成
	createBody();							//ロボット生成

	// run simulation
	dsSimulationLoop (argc,argv,640,640,&fn);

	//後始末
	destroyBot ();
	dSpaceDestroy (space);
	dWorldDestroy (world);
	return 0;
}