classdef LWRInput < SingletonCoordinateFrame
  % lwr input coordinate frame
  methods
    function obj=LWRInput(r)
      typecheck(r,{'TimeSteppingRigidBodyManipulator','RigidBodyManipulator'});

      manipInputFrame = r.getManipulator().getInputFrame();
      if (r.hands > 0)
        manipInputFrame = manipInputFrame.getFrameByNum(1);
      end
      input_names = manipInputFrame.getCoordinateNames();
      %input_names = regexprep(input_names,'_motor',''); % remove motor suffix     
      
      obj = obj@SingletonCoordinateFrame('LWRInput',length(input_names),'x',input_names);
    end
  end
end
