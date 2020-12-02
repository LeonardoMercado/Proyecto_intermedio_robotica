rosinit;
imgSub=rossubscriber('/camera/image_data');
receive(imgSub,10);
[velPub,velMsg]=rospublisher('/mobile_base_simple/goal');
vidPlayer=vision.DeployableVideoPlayer;
params=controlParamsGazebo;