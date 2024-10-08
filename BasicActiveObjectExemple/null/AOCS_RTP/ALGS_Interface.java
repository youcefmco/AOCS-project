// --------------------------------------------------------
// Code generated by Papyrus Java
// --------------------------------------------------------

package AOCS_RTP;

/************************************************************/
/**
 * the AOCS algorithms
 */
public class ALGS_Interface {
	/**
	 * 
	 */
	public class ALGS_TCMD {
		/**
		 * Torquer number
		  * 
		 */
		public int torquerID;
		/**
		 * requested new value
		 */
		public int torquerValue;
		/**
		 * enable or disable
		 */
		public boolean mtqEnableFlag;
		/**
		 * wheel number
		 */
		public int wheelID;
		/**
		 * requested speed
		 */
		public int wheelValue;
		/**
		 * enable or disable the wheel
		 */
		public boolean whlEnableFlag;
	};

	/**
	 * The AOCS structure used to pass data between the shell and the AOCS algorithms
	 */
	public class ALGS_Struct {
		/**
		 * 
		 */
		public boolean mode_isReset;
		/**
		 * 
		 */
		public AOCS_RTP.AOCS_State_Machine.AOCS_MODES mode_current;
		/**
		 * 
		 */
		public boolean mode_isSafeMode;
		/**
		 * 
		 */
		public boolean mode_isUnsolicited;
		/**
		 * 
		 */
		public int test_configuration;
		/**
		 * 
		 */
		public boolean enableSns;
		/**
		 * 
		 */
		public boolean enableAct;
	};

	/**
	 * 
	 */
	public ALGS_Struct algs_Struct;
	/**
	 * 
	 */
	public ALGS_TCMD algs_tcmd;

	/**
	 * RUN THE AOCS ALGORITHMS 
	 */
	public void AINT_AlgManager() {
		int callVal = 0;

		/* run the algorithms AJ - change prototype */
		callVal = AOCS_Go();

		/* update the alg telemetry */
		AOCS_Telemetry();

		/* check if algorithms ran successfully */
		if (callVal != 0) {
			System.out.println("Algorithm Error");
		}
	}

	/**
	 * update the alg telemetry 
	 */
	public void AOCS_Telemetry() {
	}

	/**
	 * run the algorithms
	 * @return 
	 */
	public int AOCS_Go() {
		//TODO implement the algs here
		return 0;
	}

	/**
	 * initialise AOCS variables etc.
	 */
	public void AINT_Init() {
	}
}
