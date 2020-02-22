% FUNCTION NAME: newtonEuler(linkList, paramList, paramListDot,
% paramListDDot, boundry_conditions ) to find Jv,Jvdot and jointtorques
% 
%  [jointTorques, Jv, JvDot] = newtonEuler(linkList, paramList,
%  paramListDot, paramListDDot, boundry_conditions )to find Jv,Jvdot and
%  joint torques
% 
% jointTorques = joint torques
% Jv = Jacobian Matrix
% JvDot = Jacobian_Dot Matrix
%
%
%
% linkList = link parameters
% paramList = varying parameters
% paramListDot = Differentiation of varying parameters
% paramListDDot = Double Differentiation of varying parameters
% boundary_conditions = boundary conditions structure
% base_angular_velocity      
% base_angular_acceleration      
% base_linear_acceleration (add gravity in here)      
% distal_force      
% distal_torque
%
%
% 
% Vicknesh
% 10847953 
% MEGN544 
% 10-11-2018 (edited on 11-28-2018)

function [jointTorques,Jv,JvDot] = newtonEuler( linkList, paramList,paramListDot, paramListDDot, boundry_conditions )

    numJoints = length(linkList);
    list = repmat(struct( 'zlast', zeros(3,1),'woi', zeros(3,1),'doi', zeros(3,1),'doi_dot', zeros(3,1),...	
                          'Hi', eye(4),'Fi', zeros(3,1),'Ni', zeros(3,1) ),numJoints, 1 );
    
    Toi = eye(4);
    v = zeros(3,1);
	w_base = boundry_conditions.base_angular_velocity;
    
    w = w_base; 
    w_dot = boundry_conditions.base_angular_acceleration; 
    v_dot = boundry_conditions.base_linear_acceleration; 

    for i = 1 : numJoints
        link = linkList(i);
        H = dhFwdKine(link, paramList(i));

        z = Toi(1:3,3); 
        dprev = Toi(1:3,4);
        Toi = Toi*H;
        d0_sub = Toi(1:3,4) - dprev;
        r0_sub = Toi(1:3,1:3)*(H(1:3,1:3)'*H(1:3,4) + link.com);
        
        if link.isRotary
            w_dot = w_dot + paramListDDot(i)*z + paramListDot(i)*cross(w, z);
            w = w + paramListDot(i)*z;
            vcdot = v_dot + cross(w_dot, r0_sub) + cross( w, cross(w, r0_sub) );
            v_dot = v_dot + cross(w_dot, d0_sub) + cross( w, cross(w, d0_sub) );
            v = v + cross(w, d0_sub);
        else
            vcdot = v_dot+cross(w_dot,r0_sub)+cross(w,cross(w,r0_sub)) +paramListDDot(i)*z + 2*paramListDot(i)*cross(w, z);
            v_dot = v_dot+cross(w_dot,d0_sub)+cross(w,cross(w,d0_sub))+paramListDDot(i)*z+2*paramListDot(i)*cross(w,z);
            v = v + cross(w, d0_sub) + paramListDot(i)*z;
        end
        
        Rt = Toi(1:3,1:3)';
        ivcdot = Rt*vcdot;
        iw = Rt*w;
        iwdot = Rt*w_dot;
        f = link.mass*(ivcdot);
        n = link.inertia*(iwdot) + cross(iw, link.inertia*iw);

        list(i).doi = Toi(1:3,4);
        list(i).woi = w - w_base;
        list(i).doi_dot = v - cross(w_base, list(i).doi);
        list(i).zlast = z;
        
        list(i).Hi = H;
        list(i).Fi = f;
        list(i).Ni = n;
    end

    Jv = zeros(6, numJoints);
    JvDot = zeros(6, numJoints);
    doN = list(end).doi;
    voN = list(end).doi_dot;

    F = boundry_conditions.distal_force;
    N = boundry_conditions.distal_torque;
    jointTorques = zeros(numJoints, 1);
    for i = numJoints:-1:1
        if i == numJoints
            Rii1 = eye(3);
        else
            Rii1 = list(i+1).Hi(1:3,1:3);
        end
        R = list(i).Hi(1:3,1:3);
        F = list(i).Fi + Rii1*F;
        N = list(i).Ni + Rii1*N + cross( R'*list(i).Hi(1:3,4), F ) + cross(linkList(i).com, list(i).Fi);
        
        if i > 1
            diN = doN - list(i-1).doi;
            viN = voN - list(i-1).doi_dot;
        else
            diN = doN;
            viN = voN;
        end
        if linkList(i).isRotary
            jointTorques(i,1) = dot( [0;0;1], R*N ); 
            Jv(1:3,i) = cross( list(i).zlast, diN );
            Jv(4:6,i) = list(i).zlast;
            JvDot(1:3,i) = cross( cross(list(i).woi, list(i).zlast), diN ) + cross( list(i).zlast, viN );
            JvDot(4:6,i) = cross( list(i).woi, list(i).zlast );
        else
            jointTorques(i,1) = dot( [0;0;1], R*F ); 
            Jv(1:3,i) = list(i).zlast;
            Jv(4:6,i) = zeros(3,1);
            JvDot(1:3,i) = cross( list(i).woi, list(i).zlast );
            JvDot(4:6,i) = zeros(3,1);
        end
    end
end