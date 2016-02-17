classdef LWRState < SingletonCoordinateFrame
  
  methods
    function obj=LWRState(r)
      typecheck(r,{'TimeSteppingRigidBodyManipulator','RigidBodyManipulator'});
      manipStateFrame = r.getManipulator().getStateFrame();
      if (r.hands > 0)
        manipStateFrame = manipStateFrame.getFrameByNum(1);
      end
      coordinates = manipStateFrame.getCoordinateNames();
      obj = obj@SingletonCoordinateFrame('LWRState',length(coordinates),'x',coordinates);
    end
  end
end
