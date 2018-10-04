package application;


import java.io.BufferedReader;
import java.io.DataOutputStream;
import java.io.InputStreamReader;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.ArrayList;
import java.util.Date;

import javax.inject.Inject;

import com.kuka.common.ThreadUtil;
import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.conditionModel.BooleanIOCondition;
import com.kuka.roboticsAPI.conditionModel.ConditionObserver;
import com.kuka.roboticsAPI.conditionModel.ForceCondition;
import com.kuka.roboticsAPI.conditionModel.ICondition;
import com.kuka.roboticsAPI.conditionModel.IRisingEdgeListener;
import com.kuka.roboticsAPI.conditionModel.NotificationType;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.geometricModel.math.CoordinateAxis;
import com.kuka.roboticsAPI.ioModel.AbstractIO;
import com.kuka.roboticsAPI.motionModel.MotionBatch;
import com.kuka.roboticsAPI.motionModel.Spline;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;

/**
 * Implementation of a robot application.
 * <p>
 * The application provides a {@link RoboticsAPITask#initialize()} and a 
 * {@link RoboticsAPITask#run()} method, which will be called successively in 
 * the application lifecycle. The application will terminate automatically after 
 * the {@link RoboticsAPITask#run()} method has finished or after stopping the 
 * task. The {@link RoboticsAPITask#dispose()} method will be called, even if an 
 * exception is thrown during initialization or run. 
 * <p>
 * <b>It is imperative to call <code>super.dispose()</code> when overriding the 
 * {@link RoboticsAPITask#dispose()} method.</b> 
 * 
 * @see UseRoboticsAPIContext
 * @see #initialize()
 * @see #run()
 * @see #dispose()
 */
public class RobotApplication extends RoboticsAPIApplication {
	@Inject
	private Controller kuka_Sunrise_Cabinet_1;
	private LBR lbr_iiwa_7_R800_1;
	private Tool tool;

	public ArrayList<Frame> _trackPoints;
	private ArrayList<Spline> _track;
	private ConditionObserver trackServer;

	private ICondition forceCon;
	private CartesianImpedanceControlMode soft;
	private Frame frames[];
	private MotionBatch mb[];
	private ObjectFrame world;
	private ObjectFrame actTCP;
	private MediaFlangeIOGroup mediaIO;
	private ForceCondition gestureForce;
	private double gestureForceVal;

	@Override
	public void initialize() {
		kuka_Sunrise_Cabinet_1 = getController("KUKA_Sunrise_Cabinet_1");
		lbr_iiwa_7_R800_1 = (LBR) getDevice(kuka_Sunrise_Cabinet_1,
				"LBR_iiwa_7_R800_1");

		// actTCP = lbr_iiwa_14_R820_1.getFlange();

		// init TCP
		tool = createFromTemplate("Pin");
		tool.attachTo(lbr_iiwa_7_R800_1.getFlange());
		actTCP = tool.getFrame("/Pin_TCP");

		_track = new ArrayList<Spline>();
		_trackPoints = new ArrayList<Frame>();


		mediaIO = new MediaFlangeIOGroup(kuka_Sunrise_Cabinet_1);

		soft = new CartesianImpedanceControlMode();
		soft.parametrize(CartDOF.ALL).setDamping(.7);
		soft.parametrize(CartDOF.ROT).setStiffness(100);
		soft.parametrize(CartDOF.TRANSL).setStiffness(600);

		// World-Frame
		world = World.Current.getRootFrame();
		lbr_iiwa_7_R800_1.setESMState("2");

		// init Stop-Funktion
		AbstractIO UserButton = mediaIO.getInput("UserButton");
		BooleanIOCondition bCond = new BooleanIOCondition(UserButton, true);
		IRisingEdgeListener trackListen = new IRisingEdgeListener() {

			@Override
			public void onRisingEdge(ConditionObserver conditionObserver,
					Date time, int missedEvents) {
				getApplicationData().getProcessData("trackPath").setValue(true);
			}
		};

		trackServer = getObserverManager().createAndEnableConditionObserver(
				bCond, NotificationType.EdgesOnly, trackListen);

		// Force condition for gesture control
		gestureForceVal = getApplicationData().getProcessData("gestureForce")
				.getValue();

		gestureForce = ForceCondition.createNormalForceCondition(
				lbr_iiwa_7_R800_1.getFlange(), CoordinateAxis.Y,
				gestureForceVal);

	}

	@Override
	public void run() throws Exception{
		// your application execution starts here
		// lBR_iiwa_7_R800_1.move(ptpHome());
		mediaIO.setLEDBlue(true);
		ThreadUtil.milliSleep(1000);
		mediaIO.setLEDBlue(false);
		ThreadUtil.milliSleep(1000);
		String clientSentence;
		ServerSocket welcomeSocket = new ServerSocket(30005);
		mediaIO.setLEDBlue(true);
		ThreadUtil.milliSleep(1000);
		mediaIO.setLEDBlue(false);
		ThreadUtil.milliSleep(1000);
		Socket connectionSocket = welcomeSocket.accept();
		mediaIO.setLEDBlue(true);
		ThreadUtil.milliSleep(1000);
		mediaIO.setLEDBlue(false);
		ThreadUtil.milliSleep(1000);
		
		
		BufferedReader inFromClient = new BufferedReader(new InputStreamReader(connectionSocket.getInputStream()));
		DataOutputStream outToClient = new DataOutputStream(connectionSocket.getOutputStream());

//		boolean active = true;
		
//		while (active) {
//		   clientSentence = inFromClient.readLine();
//			mediaIO.setLEDBlue(true);
//			ThreadUtil.milliSleep(1000);
//			mediaIO.setLEDBlue(false);
//			
//			getLogger().info(clientSentence);
//			if(clientSentence.equals("exit")){
//				active = false;
//			} else{
//				outToClient.writeBytes(clientSentence + "\n");
//				getLogger().info("after write byte");
//			}
//	  }
			

		boolean active = true;
		while (active){
			clientSentence = inFromClient.readLine();
			int movement = Integer.parseInt(clientSentence);
			getLogger().info(clientSentence);
			outToClient.writeChars(clientSentence);
			//switch movement cases
	        switch (movement) {
	        	case 0:
	        		active = false;
		    		outToClient.writeBytes("Default application ending.\n");
	        		break;
	        		
	            case 1:
		    		lbr_iiwa_7_R800_1.move(ptp(getApplicationData().getFrame("/StartPos")).setJointVelocityRel(0.25));
		    		
		    		mediaIO.setLEDBlue(true);
		    		ThreadUtil.milliSleep(1000);
		    		mediaIO.setLEDBlue(false);
		    						
		    		
		    		getLogger().info("Before" + System.currentTimeMillis());
		    		lbr_iiwa_7_R800_1.move(ptp(getApplicationData().getFrame("/P1")).setJointVelocityRel(0.25));
		    		getLogger().info("After" + System.currentTimeMillis());
		    		lbr_iiwa_7_R800_1.move(ptp(getApplicationData().getFrame("/StartPos")).setJointVelocityRel(0.25));
		    					
		    		mediaIO.setLEDBlue(true);
		    		ThreadUtil	.milliSleep(1000);
		    		mediaIO.setLEDBlue(false);
		    		outToClient.writeBytes("Movement 1 done.\n");
                    break;
                    
	            case 2:
		    		// your application execution starts here
		    		// lBR_iiwa_7_R800_1.move(ptpHome());
		    		lbr_iiwa_7_R800_1.move(ptp(getApplicationData().getFrame("/StartPos")).setJointVelocityRel(0.25));
		    		
		    		mediaIO.setLEDBlue(true);
		    		ThreadUtil.milliSleep(1000);
		    		mediaIO.setLEDBlue(false);
		    						
		    		lbr_iiwa_7_R800_1.move(ptp(getApplicationData().getFrame("/P3")).setJointVelocityRel(0.25));
		    		lbr_iiwa_7_R800_1.move(ptp(getApplicationData().getFrame("/StartPos")).setJointVelocityRel(0.25));
		   		
		    		mediaIO.setLEDBlue(true);
		    		ThreadUtil.milliSleep(1000);
		    		mediaIO.setLEDBlue(false);
		    		// lbr_iiwa_7_R800_1.move(ptpHome());
		    		// lbr_iiwa_7_R800_1.move(ptp(0.0, 0.0, 0.0, 1.57, 0.0, -1.57, 0.0));
		    		outToClient.writeBytes("Movement 2 done.\n");
		    		break;
	                    
			 }
		}
		   
	 }
}

