
% disable JxBrowser to prevent idle cpu use
% com.mathworks.mlwidgets.html.HtmlComponentFactory.setDefaultType('HTMLRENDERER');
% 
% addpath(genpath([pwd,'/../../HSCC2016']));
% addpath(genpath([pwd,'/../../HSCC2016/lib/drake-distro/spotless']));
% addpath([pwd,'/../../HSCC2016/lib/drake-distro/drake/examples/DubinsCar']);
% addpath([pwd,'/../../HSCC2016/lib/drake-distro/drake']);
% 
% addpath(genpath('/home/jon/Software/mosek'));
% 
% run('/home/jon/Software/drake-distro/drake/addpath_drake');


addpath(genpath([pwd,'/../../HSCC2016']));
rmpath(genpath([pwd,'/../../HSCC2016/lib/drake-distro-lcm-win64']));
run('/home/jon/Software/drake-distro/drake/addpath_drake');
addpath(genpath('/home/jon/Software/mosek'));
addpath(genpath('/home/jon/Software/drake-distro/spotless'));
addpath('/home/jon/Software/drake-distro/drake/examples/DubinsCar');
addpath('/home/jon/Software/drake-distro/drake');
rmpath([pwd,'/../../HSCC2016/lib/drake-distro-lcm-win64/spotless']);
rmpath([pwd,'/../../HSCC2016/lib/spotless-master']);

