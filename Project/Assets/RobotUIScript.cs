using System.Collections.Generic;
using UnityEngine;
using System.Collections; // This contains IEnumerator
using TMPro; // Required for TextMeshPro


public class RobotUIScript : MonoBehaviour
{
    // Drag your 6 Joint GameObjects here in the Inspector
    public Transform[] robotJoints; 
    public JointGrapher grapher; // Drag the GraphContainer here
    private Vector3[] baseEulerAngles;
    public float[] MaxLimit;
    private string errorMessage = "";
    private bool showMessage = false;
    public float[] MinLimit;
    private float[] MaxLimitDeg;
    private float[] MinLimitDeg;
    public TextMeshProUGUI modeButtonText; // Drag the Button's Text object here
    public TextMeshProUGUI IntButtonText; // Drag the Button's Text object here
    private bool isMatrixMode = false;
    private bool isIntModeMoveL = false;
    [Header("UI Panels")]
    public GameObject jointInputsPanelStart;  // Drag the parent of your 6 joint inputs here
    public GameObject matrixInputsPanelStart;
    public GameObject jointInputsPanelEnd;  // Drag the parent of your 6 joint inputs here
    public GameObject matrixInputsPanelEnd;
    // Drag your 6 Input Fields here in the Inspector
    public TMP_InputField[] JointInputsStart;
    public TMP_InputField[] MatrixInputsStart;
    public TMP_InputField[] JointInputsEnd;
    public TMP_InputField[] MatrixInputsEnd;
    void Start()
{
    // Initialize the array to match the number of joints
    baseEulerAngles = new Vector3[robotJoints.Length];
    modeButtonText.text = "MODE: JOINT ANGLES"; // Set initial button text
    IntButtonText.text = "MoveJ"; // Set initial button text
    jointInputsPanelStart.SetActive(true);
    matrixInputsPanelStart.SetActive(false);
    jointInputsPanelEnd.SetActive(true);
    matrixInputsPanelEnd.SetActive(false);
    MaxLimitDeg=new float[6];
    MinLimitDeg=new float[6];
    for(int i=0;i<6;i++)
    {
        MaxLimitDeg[i]=MaxLimit[i]*180/Mathf.PI;
        MinLimitDeg[i]=MinLimit[i]*180/Mathf.PI;
    }
    //jointInputsPanel.visible = true;
    //matrixInputsPanel.visible = false;
    for (int i = 0; i < robotJoints.Length; i++)
    {
        // We capture the "Setup" rotation (X, Y, and Z) 
        // that you painstakingly aligned in the editor.
        baseEulerAngles[i] = robotJoints[i].localEulerAngles;
    }
}
public void ToggleInputMode()
    {
        // 1. Flip the true/false switch
        isMatrixMode = !isMatrixMode;

        // 2. Update the button's text based on the new state
        if (isMatrixMode)
        {
            modeButtonText.text = "MODE: MATRIX (HTM)";
            jointInputsPanelStart.SetActive(false);
            matrixInputsPanelStart.SetActive(true);
            jointInputsPanelEnd.SetActive(false);
            matrixInputsPanelEnd.SetActive(true);
            //jointInputsPanel.visible = false;
            //matrixInputsPanel.visible = true;

            // Add code here to show your Matrix Panel
        }
        else
        {
            modeButtonText.text = "MODE: JOINT ANGLES";
            jointInputsPanelStart.SetActive(true);
            matrixInputsPanelStart.SetActive(false);
            jointInputsPanelEnd.SetActive(true);
            matrixInputsPanelEnd.SetActive(false);
            //jointInputsPanel.visible = true;
            //matrixInputsPanel.visible = false;
            // Add code here to show your Joint Panel
        }
        
        Debug.Log("Switched to: " + modeButtonText.text);
    }
    public void Animate()
    {
        print("Animate button clicked!");
        print("Current Mode - Matrix: " + isMatrixMode + ", MoveL: " + isIntModeMoveL);
        float[,] Angles=new float[6,100];
        if(isMatrixMode && isIntModeMoveL)
        {
            Angles=MoveLMat()      ;
                  print("Animating MoveJMat...");

        }
        else if(isMatrixMode && !isIntModeMoveL)
        {
            print("Animating MoveJMat...");
            Angles=MoveJMat();
        }
        else if(!isMatrixMode && isIntModeMoveL)
        {
            Angles=MoveLAngles();
        }
        else if(!isMatrixMode && !isIntModeMoveL)
        {
            Angles=MoveJJointAngles();
        }
        grapher.UpdateGraph(Angles);
        StopAllCoroutines();
        StartCoroutine(PlayMotionPath(Angles)); 
    }
    public void ToggleIntMode()
    {
        // 1. Flip the true/false switch
        isIntModeMoveL = !isIntModeMoveL;

        // 2. Update the button's text based on the new state
        if (isIntModeMoveL)
        {
            IntButtonText.text = "MoveL";
            // Add code here to show your Matrix Panel
        }
        else
        {
            IntButtonText.text = "MoveJ";
            // Add code here to show your Joint Panel
        }
        
        Debug.Log("Switched to: " + IntButtonText.text);
    }
    public float[,] MoveJJointAngles()
    {
        int stepnum=100;
        float[,] CoefMat= new float[6,4];
        float[,] ThetaArr=new float[6,stepnum];
        float StartVel=0;
        float EndVel=0;
        float tf=1;
        float StartAngle=0;
        float EndAngle=0;
        for (int i = 0; i < robotJoints.Length; i++)
        {
            float.TryParse(JointInputsStart[i].text,out StartAngle);
            float.TryParse(JointInputsEnd[i].text,out EndAngle);

            
            CoefMat[i,0]=StartAngle;
            CoefMat[i,1]=StartVel;
            CoefMat[i,2]=(3/(tf*tf))*(EndAngle-StartAngle);
            CoefMat[i,3]=(-2/(tf*tf*tf))*(EndAngle-StartAngle);
        }
        for(int i=0;i<stepnum;i++)
        {
            for (int j = 0; j < robotJoints.Length; j++){
                ThetaArr[j,i]=CoefMat[j,0]+CoefMat[j,1]*i*tf/stepnum+CoefMat[j,2]*Mathf.Pow(i*tf/stepnum,2)+CoefMat[j,3]*Mathf.Pow(i*tf/stepnum,3);
            }
        }
        return ThetaArr;

    }
    public float[,] MoveJMat()
    {

        float[] MatrixInputsStartFloats= new float[12];
        for(int i=0;i<MatrixInputsStart.Length;i++)
        {
            float.TryParse(MatrixInputsStart[i].text,out MatrixInputsStartFloats[i]);
        }
        Matrix4x4 HT= new Matrix4x4();
        HT[0, 0] = MatrixInputsStartFloats[0]; HT[0, 1] = MatrixInputsStartFloats[1]; HT[0, 2] = MatrixInputsStartFloats[2]; HT[0, 3]  = MatrixInputsStartFloats[3];
        // Row 1
        HT[1, 0] = MatrixInputsStartFloats[4]; HT[1, 1] = MatrixInputsStartFloats[5]; HT[1, 2] = MatrixInputsStartFloats[6]; HT[1, 3]  = MatrixInputsStartFloats[7];
        // Row 2
        HT[2, 0] = MatrixInputsStartFloats[8]; HT[2, 1] = MatrixInputsStartFloats[9]; HT[2, 2] = MatrixInputsStartFloats[10]; HT[2, 3]  = MatrixInputsStartFloats[11];
        // Row 3 (HoGogeneous coordinate - usually 0, 0, 0, 1)
        HT[3, 0] = 0; HT[3, 1] = 0; HT[3, 2] = 0; HT[3, 3]  = 1;

        List<float[]> StartAngleList=IK(HT);


        float[] MatrixInputsEndFloats= new float[12];
        for(int i=0;i<MatrixInputsEndFloats.Length;i++)
        {
            float.TryParse(MatrixInputsEnd[i].text,out MatrixInputsEndFloats[i]);
        }
        Matrix4x4 HT2= new Matrix4x4();
        HT2[0, 0] = MatrixInputsEndFloats[0]; HT2[0, 1] = MatrixInputsEndFloats[1]; HT2[0, 2] = MatrixInputsEndFloats[2]; HT2[0, 3]  = MatrixInputsEndFloats[3];
        // Row 1
        HT2[1, 0] = MatrixInputsEndFloats[4]; HT2[1, 1] = MatrixInputsEndFloats[5]; HT2[1, 2] = MatrixInputsEndFloats[6]; HT2[1, 3]  = MatrixInputsEndFloats[7];
        // Row 2
        HT2[2, 0] = MatrixInputsEndFloats[8]; HT2[2, 1] = MatrixInputsEndFloats[9]; HT2[2, 2] = MatrixInputsEndFloats[10]; HT2[2, 3]  = MatrixInputsEndFloats[11];
        // Row 3 (HoGogeneous coordinate - usually 0, 0, 0, 1)
        HT2[3, 0] = 0; HT2[3, 1] = 0; HT2[3, 2] = 0; HT2[3, 3]  = 1;

        List<float[]> EndAngleList=IK(HT2);

        float mindist= Mathf.Infinity;
        float[] StartAngles= new float[6];
        float[] EndAngles= new float[6];
        foreach(float[] startangle in StartAngleList)
        {
            foreach(float[] endangle in EndAngleList)
            {
                float dist=0;
                for(int i=0;i<6;i++)                {
                    dist+=Mathf.Pow(endangle[i]-startangle[i],2);
                }
                dist=Mathf.Sqrt(dist);
                if(dist<mindist)                {
                    mindist=dist;
                    StartAngles=startangle;
                    EndAngles=endangle;
                }
            } 
        }      
        print("Chosen Start Angles: ");
        for(int i=0;i<StartAngles.Length;i++)        {
            StartAngles[i]=StartAngles[i]*180/Mathf.PI;
            print("Theta"+(i+1)+": "+StartAngles[i]);
        }
        print("Chosen End Angles: ");
        for(int i=0;i<EndAngles.Length;i++)        {
            EndAngles[i]=EndAngles[i]*180/Mathf.PI;
            print("Theta"+(i+1)+": "+EndAngles[i]);
        }




        int stepnum=100;
        float[,] CoefMat= new float[6,4];
        float[,] ThetaArr=new float[6,stepnum];
        float StartVel=0;
        float EndVel=0;
        float tf=1;
        float StartAngle=0;
        float EndAngle=0;
        for (int i = 0; i < robotJoints.Length; i++)
        {
            StartAngle=StartAngles[i];
            EndAngle=EndAngles[i];

            
            CoefMat[i,0]=StartAngle;
            CoefMat[i,1]=StartVel;
            CoefMat[i,2]=(3/(tf*tf))*(EndAngle-StartAngle);
            CoefMat[i,3]=(-2/(tf*tf*tf))*(EndAngle-StartAngle);
        }
        for(int i=0;i<stepnum;i++)
        {
            for (int j = 0; j < robotJoints.Length; j++){
                ThetaArr[j,i]=CoefMat[j,0]+CoefMat[j,1]*i*tf/stepnum+CoefMat[j,2]*Mathf.Pow(i*tf/stepnum,2)+CoefMat[j,3]*Mathf.Pow(i*tf/stepnum,3);
            }
        }
        return ThetaArr;

    }

    public float[,] MoveLMat()
    {
        float steps=250;
       
        float[] MatrixInputsStartFloats= new float[12];
        for(int i=0;i<MatrixInputsStart.Length;i++)
        {
            float.TryParse(MatrixInputsStart[i].text,out MatrixInputsStartFloats[i]);
        }
        Matrix4x4 HT= new Matrix4x4();
        HT[0, 0] = MatrixInputsStartFloats[0]; HT[0, 1] = MatrixInputsStartFloats[1]; HT[0, 2] = MatrixInputsStartFloats[2]; HT[0, 3]  = MatrixInputsStartFloats[3];
        // Row 1
        HT[1, 0] = MatrixInputsStartFloats[4]; HT[1, 1] = MatrixInputsStartFloats[5]; HT[1, 2] = MatrixInputsStartFloats[6]; HT[1, 3]  = MatrixInputsStartFloats[7];
        // Row 2
        HT[2, 0] = MatrixInputsStartFloats[8]; HT[2, 1] = MatrixInputsStartFloats[9]; HT[2, 2] = MatrixInputsStartFloats[10]; HT[2, 3]  = MatrixInputsStartFloats[11];
        // Row 3 (HoGogeneous coordinate - usually 0, 0, 0, 1)
        HT[3, 0] = 0; HT[3, 1] = 0; HT[3, 2] = 0; HT[3, 3]  = 1;
        List<float[]> StartjointAngles = IK(HT);
        for(int l=0;l<StartjointAngles.Count;l++)
        {
            print("Start Joint Angles for Config "+l+": ");
            for(int m=0;m<StartjointAngles[l].Length;m++)
            {
                print("Theta"+(m+1)+": "+StartjointAngles[l][m]*180/Mathf.PI+" Start Joint Angles for Config");
            }
        }



        float[] MatrixInputsEndFloats= new float[12];
        for(int i=0;i<MatrixInputsEndFloats.Length;i++)
        {
            float.TryParse(MatrixInputsEnd[i].text,out MatrixInputsEndFloats[i]);
        }
        Matrix4x4 HT2= new Matrix4x4();
        HT2[0, 0] = MatrixInputsEndFloats[0]; HT2[0, 1] = MatrixInputsEndFloats[1]; HT2[0, 2] = MatrixInputsEndFloats[2]; HT2[0, 3]  = MatrixInputsEndFloats[3];
        // Row 1
        HT2[1, 0] = MatrixInputsEndFloats[4]; HT2[1, 1] = MatrixInputsEndFloats[5]; HT2[1, 2] = MatrixInputsEndFloats[6]; HT2[1, 3]  = MatrixInputsEndFloats[7];
        // Row 2
        HT2[2, 0] = MatrixInputsEndFloats[8]; HT2[2, 1] = MatrixInputsEndFloats[9]; HT2[2, 2] = MatrixInputsEndFloats[10]; HT2[2, 3]  = MatrixInputsEndFloats[11];
        // Row 3 (HoGogeneous coordinate - usually 0, 0, 0, 1)
        HT2[3, 0] = 0; HT2[3, 1] = 0; HT2[3, 2] = 0; HT2[3, 3]  = 1;
        List<float[]> EndAngleList=IK(HT2);

        Matrix4x4 StartMatrix=HT;
        Matrix4x4 EndMatrix=HT2;
        float[,] Path=new float[6,(int)steps];
        bool invalidSolution=false;
        print("StartjointAngles Count: "+StartjointAngles.Count);
        for(int ConfigInt=0; ConfigInt<8;ConfigInt++)
        {
            invalidSolution=false;
            print("ConfigInt: "+ConfigInt);

            float[] StartAngle=StartjointAngles[ConfigInt];
            float[] EndAngle=EndAngleList[ConfigInt];     
            
            // 1. Extract start/end positions and rotations
            Vector3 startPos = StartMatrix.GetColumn(3);
            Vector3 endPos = EndMatrix.GetColumn(3);
            float[] StepAngle= new float[6];
            Quaternion startRot = StartMatrix.rotation;
            Quaternion endRot = EndMatrix.rotation;
            float[] LastAngle= new float[6];
            Path[0,0]=StartAngle[0]*180/Mathf.PI;
            Path[1,0]=StartAngle[1]*180/Mathf.PI;  
            Path[2,0]=StartAngle[2]*180/Mathf.PI;
            Path[3,0]=StartAngle[3]*180/Mathf.PI;
            Path[4,0]=StartAngle[4]*180/Mathf.PI;
            Path[5,0]=StartAngle[5]*180/Mathf.PI;
            LastAngle=StartAngle;
            for (int i = 1; i < steps; i++)
            {
                float t = (float)i / steps;

                // 2. Interpolate Position and Rotation
                Vector3 currentPos = Vector3.Lerp(startPos, endPos, t);
                Quaternion currentRot = Quaternion.Slerp(startRot, endRot, t);

                // 3. Reconstruct the Matrix for this step
                Matrix4x4 currentTargetMatrix = Matrix4x4.TRS(currentPos, currentRot, Vector3.one);

                // 4. Run your Inverse Kinematics function
                // This needs to return the 6 angles for this specific 3D point
                List<float[]> StepjointAngles = IK(currentTargetMatrix);
                print(StepjointAngles.Count);
                StepAngle=StepjointAngles[ConfigInt];
                print("Step "+i+" Angles: "+"Theta"+(1)+": "+StepAngle[0]*180/Mathf.PI+", Theta"+(2)+": "+StepAngle[1]*180/Mathf.PI+", Theta"+(3)+": "+StepAngle[2]*180/Mathf.PI+", Theta"+(4)+": "+StepAngle[3]*180/Mathf.PI+", Theta"+(5)+": "+StepAngle[4]*180/Mathf.PI+", Theta"+(6)+": "+StepAngle[5]*180/Mathf.PI);                        
                if(float.IsNaN(StepAngle[0])||float.IsNaN(StepAngle[1])||float.IsNaN(StepAngle[2])||float.IsNaN(StepAngle[3])||float.IsNaN(StepAngle[4])||float.IsNaN(StepAngle[5]))
                {
                    invalidSolution=true;
                    
                    break;
                }
                    
                Path[0,i]=StepAngle[0]*180/Mathf.PI;
                Path[1,i]=StepAngle[1]*180/Mathf.PI;
                Path[2,i]=StepAngle[2]*180/Mathf.PI;
                Path[3,i]=StepAngle[3]*180/Mathf.PI;
                Path[4,i]=StepAngle[4]*180/Mathf.PI;
                Path[5,i]=StepAngle[5]*180/Mathf.PI;
            }
            if(invalidSolution==false)
            {
                break;
            }
        } 
        if(invalidSolution==true){
            Debug.LogError("NoValidSolution.");
            TriggerError("NoValidSolution");
            float[,] emptyPath= new float[6,1];
            return emptyPath;
        }  
        return Path;
        
       
    }

    public float[,] MoveLAngles()
    {
        float[] InputStartAngle= new float[6];
        float[] InputEndAngle= new float[6];
        for (int i = 0; i < robotJoints.Length; i++)
        {
            float.TryParse(JointInputsStart[i].text,out InputStartAngle[i]);
            float.TryParse(JointInputsEnd[i].text,out InputEndAngle[i]);
            InputStartAngle[i]=InputStartAngle[i]*Mathf.PI/180;
            InputEndAngle[i]=InputEndAngle[i]*Mathf.PI/180; 

        }
        Matrix4x4 HT=FK(InputStartAngle);
        print("HT: "+HT);
        Matrix4x4 HT2=FK(InputEndAngle);

        float steps=250;
        int ConfigInt=10;
        float tolerance=.05f;
        List<float[]> StartjointAnglesConfigs = IK(HT);

       for (int col=0;col<8;col++)
        {
            float[] target=StartjointAnglesConfigs[col];
            bool isMatch = true;
            print(InputStartAngle[0]+", "+InputStartAngle[1]+", "+InputStartAngle[2]+", ewq"+InputStartAngle[3]+", "+InputStartAngle[4]+", "+InputStartAngle[5]);
            print(target[0]+", "+target[1]+", "+target[2]+", "+target[3]+", "+target[4]+", qweewq"+target[5]);
            for (int row = 0; row < 6; row++)
            {
                print("MODTEST: "+TrueModulo(InputStartAngle[row],(2*Mathf.PI))+", "+TrueModulo(target[row],(2*Mathf.PI)));
                if(float.IsNaN(target[row]))
                {
                    isMatch=false;
                    break;
                }
                // Compare the absolute difference to the tolerance
                if (Mathf.Abs(TrueModulo(InputStartAngle[row],(2*Mathf.PI)) - TrueModulo(target[row],(2*Mathf.PI))) > tolerance)
                {
                    isMatch = false;
                    break; // Optimization: Stop checking this column if one joint fails
                }
            }

            if (isMatch)
            {
                ConfigInt = col;
               
                print("Matching configuration found at column: " + ConfigInt);
                break; // Found the matching configuration, exit the loop
            }
        }
        

     

        Matrix4x4 StartMatrix=HT;
        Matrix4x4 EndMatrix=HT2;
        float[,] Path=new float[6,(int)steps];
        bool invalidSolution=false;
        
            invalidSolution=false;
            print("ConfigInt: "+ConfigInt);

            
            
            // 1. Extract start/end positions and rotations
            Vector3 startPos = StartMatrix.GetColumn(3);
            Vector3 endPos = EndMatrix.GetColumn(3);
            float[] StepAngle= new float[6];
            Quaternion startRot = StartMatrix.rotation;
            Quaternion endRot = EndMatrix.rotation;
            float[] LastAngle= new float[6];
            Path[0,0]=InputStartAngle[0]*180/Mathf.PI;
            Path[1,0]=InputStartAngle[1]*180/Mathf.PI;  
            Path[2,0]=InputStartAngle[2]*180/Mathf.PI;
            Path[3,0]=InputStartAngle[3]*180/Mathf.PI;
            Path[4,0]=InputStartAngle[4]*180/Mathf.PI;
            Path[5,0]=InputStartAngle[5]*180/Mathf.PI;
            LastAngle=InputStartAngle;
            for (int i = 1; i < steps; i++)
            {
                float t = (float)i / steps;

                // 2. Interpolate Position and Rotation
                Vector3 currentPos = Vector3.Lerp(startPos, endPos, t);
                Quaternion currentRot = Quaternion.Slerp(startRot, endRot, t);

                // 3. Reconstruct the Matrix for this step
                Matrix4x4 currentTargetMatrix = Matrix4x4.TRS(currentPos, currentRot, Vector3.one);

                // 4. Run your Inverse Kinematics function
                // This needs to return the 6 angles for this specific 3D point
                List<float[]> StepjointAngles = IK(currentTargetMatrix);
                print(StepjointAngles.Count);   
                invalidSolution=true;
                float dist=1000000000;
                for(int j=0;j<StepjointAngles.Count;j++)
                {
                    float[] TestAngle=StepjointAngles[j];
                    print("Test: "+TestAngle[0]*180/Mathf.PI+", "+TestAngle[1]*180/Mathf.PI+", "+TestAngle[2]*180/Mathf.PI+", "+TestAngle[3]*180/Mathf.PI+", "+TestAngle[4]*180/Mathf.PI+", "+TestAngle[5]*180/Mathf.PI);
                    print("LAtsTEP: "+Path[0,i-1]+", "+Path[1,i-1]+", "+Path[2,i-1]+", "+Path[3,i-1]+", "+Path[4,i-1]+", "+Path[5,i-1]);

                    if(float.IsNaN(TestAngle[0])||float.IsNaN(TestAngle[1])||float.IsNaN(TestAngle[2])||float.IsNaN(TestAngle[3])||float.IsNaN(TestAngle[4])||float.IsNaN(TestAngle[5]))
                    {
                        continue;
                    }

                    float Tempdist=0;
                    for(int k=0;k<6;k++)
                    {
                        Tempdist+=Mathf.Pow(TrueModulo(Path[k,i-1]*(Mathf.PI/180),(2*Mathf.PI)) - TrueModulo(TestAngle[k],(2*Mathf.PI)),2);
                    }
                    if(Tempdist<dist)
                    {
                        dist=Tempdist;
                        StepAngle=TestAngle;
                        invalidSolution=false;
                    }
                   
                  
                }
                print("STEPANGLE: "+StepAngle[0]*180/Mathf.PI+", "+StepAngle[1]*180/Mathf.PI+", "+StepAngle[2]*180/Mathf.PI+", "+StepAngle[3]*180/Mathf.PI+", "+StepAngle[4]*180/Mathf.PI+", "+StepAngle[5]*180/Mathf.PI);
                
                  
                Path[0,i]=StepAngle[0]*180/Mathf.PI;
                Path[1,i]=StepAngle[1]*180/Mathf.PI;
                Path[2,i]=StepAngle[2]*180/Mathf.PI;
                Path[3,i]=StepAngle[3]*180/Mathf.PI;
                Path[4,i]=StepAngle[4]*180/Mathf.PI;
                Path[5,i]=StepAngle[5]*180/Mathf.PI;
            }
            
        
        if(invalidSolution==true){
            Debug.LogError("NoValidSolution.");
            TriggerError("NoValidSolution");
            float[,] emptyPath= new float[6,1];
            return emptyPath;
        }  
        UnwrapPath(Path);
        for(int i=0;i<6;i++)
        {
            for(int j=1;j<steps;j++)
            {
                if(Mathf.Abs(Mathf.DeltaAngle(Path[i,j-1],Path[i,j]))>10)
                {
                    print("NoValidSolution");
                    TriggerError("NoValidSolution");
                                float[,] emptyPath= new float[6,1];

                    return emptyPath;
                }
            }
        }
        return Path;
        
       
    }



    
    IEnumerator PlayMotionPath(float[,] motionPath)
    {
    int totalSteps = motionPath.GetLength(1); // Gets the '100'

    for (int step = 0; step < totalSteps; step++)
    {
        // For each step, apply the angles to all 6 joints
        for (int i = 0; i < 6; i++)
        {
            float angle = motionPath[i, step];

            // Apply using your specific KUKA-style rotation logic
            robotJoints[i].localRotation = Quaternion.Euler(baseEulerAngles[i].x, baseEulerAngles[i].y, 0) 
                                           * Quaternion.Euler(0, 0, -angle);
        }

        // This is the magic line: it tells Unity "Wait until the next frame"
        yield return null; 
    }
    
    Debug.Log("Motion Complete!");
    Debug.Log("BaseEulerAngles: "+baseEulerAngles[0]+", "+baseEulerAngles[1]+", "+baseEulerAngles[2]+", "+baseEulerAngles[3]+", "+baseEulerAngles[4]+", "+baseEulerAngles[5]);
    }   
    public void UpdateRobotAngles()
    {
        for (int i = 0; i < robotJoints.Length; i++)
        {
            if (float.TryParse(JointInputsEnd[i].text, out float angle))
            {
                // 2. Clamp the value so the robot doesn't break itself
                angle = Mathf.Clamp(angle, MinLimitDeg[i], MaxLimitDeg[i]);
                JointInputsEnd[i].text = angle.ToString("F2");
            }
        }
        // Iterate through all 6 joints/input fields
        for (int i = 0; i < JointInputsStart.Length; i++)
        {
            // 1. Safely convert the string from the Input Field to a float
            if (float.TryParse(JointInputsStart[i].text, out float angle))
            {
                // 2. Clamp the value so the robot doesn't break itself
                angle = Mathf.Clamp(angle, MinLimitDeg[i], MaxLimitDeg[i]);

                // 3. Update the text field to show the clamped value (optional, but helpful)
                JointInputsStart[i].text = angle.ToString("F2");

                // 4. Apply the rotation
                // We use the 'baseEulerAngles' we saved in Start() to preserve the X and Y setup
                // And apply the user's angle to the local Z axis (inverted with -angle per your setup)
                robotJoints[i].localRotation = Quaternion.Euler(baseEulerAngles[i].x, baseEulerAngles[i].y, 0) 
                                            * Quaternion.Euler(0, 0, -angle);
            }
        }
    }

    private List<float[]> IK(Matrix4x4 HT)
    {
           
       
        float A3=.265f;
        float A2=.100f;
        float D4=.270f;
        float[] Theta1=new float[2];
        float[,] Theta2=new float[2,2];
        float[,] Theta3=new float[2,2];
        float[,,] Theta4=new float[2,2,2];
        float[,,] Theta5=new float[2,2,2];
        float[,,] Theta6=new float[2,2,2];
        Matrix4x4 TargetSphereWrist= new Matrix4x4();

        Matrix4x4 G= new Matrix4x4();
        G[0, 0] = 1; G[0, 1] = 0; G[0, 2] = 0; G[0, 3] = 0;
        // Row 1
        G[1, 0] = 0; G[1, 1] = 1; G[1, 2] = 0; G[1, 3] = 0;
        // Row 2
        G[2, 0] = 0; G[2, 1] = 0; G[2, 2] = 1; G[2, 3] = .350f;
        // Row 3 (HoGogeneous coordinate - usually 0, 0, 0, 1)
        G[3, 0] = 0; G[3, 1] = 0; G[3, 2] = 0; G[3, 3] = 1;

        Matrix4x4 H= new Matrix4x4();
        H[0, 0] = 1; H[0, 1] = 0; H[0, 2] = 0; H[0, 3] = 0;
        // Row 1
        H[1, 0] = 0; H[1, 1] = 1; H[1, 2] = 0; H[1, 3] = 0;
        // Row 2
        H[2, 0] = 0; H[2, 1] = 0; H[2, 2] = 1; H[2, 3] = .075f;
        // Row 3 (HoHoHeneous coordinate - usually 0, 0, 0, 1)
        H[3, 0] = 0; H[3, 1] = 0; H[3, 2] = 0; H[3, 3] = 1;

        TargetSphereWrist=G.inverse*HT*H.inverse;
        print("TargetSphereWrist: "+TargetSphereWrist);
        if(TargetSphereWrist[0,3]==0)
        {
            Theta1[0]=Mathf.PI/2;

        }else{
        Theta1[0]=-Mathf.Atan(TargetSphereWrist[1,3]/TargetSphereWrist[0,3]);

        }
        Theta1[1]=Theta1[0]-Mathf.PI;
        float A,B,C;
        for(int i=0;i<2;i++)
        {
        if(System.Math.Round(Mathf.Cos(Theta1[i]),2)==0)
        {
            A=-2*A3*TargetSphereWrist[2,3];
            B=2*A3*A2+(2*A3*TargetSphereWrist[1,3])/Mathf.Sin(Theta1[i]);
            C=Mathf.Pow(TargetSphereWrist[1,3]/Mathf.Sin(Theta1[i]),2)+A2*A2+A3*A3-D4*D4+TargetSphereWrist[2,3]*TargetSphereWrist[2,3]+(2*TargetSphereWrist[1,3]*A2)/Mathf.Sin(Theta1[i]);
        }
        else
        {
            
            A=-2*A3*TargetSphereWrist[2,3];
            B=2*A3*A2-(2*A3*TargetSphereWrist[0,3])/Mathf.Cos(Theta1[i]);
            C=Mathf.Pow(TargetSphereWrist[0,3]/Mathf.Cos(Theta1[i]),2)+A2*A2+A3*A3-D4*D4+TargetSphereWrist[2,3]*TargetSphereWrist[2,3]-(2*TargetSphereWrist[0,3]*A2)/Mathf.Cos(Theta1[i]);
        }
        print("A: "+A);
        print("B: "+B);
        print("C: "+C);
        print("0: "+(-B+Mathf.Sqrt(A*A+B*B-C*C))/(C-A));
        print("1: "+(-B-Mathf.Sqrt(A*A+B*B-C*C))/(C-A));
        Theta2[i,0]=2*Mathf.Atan((-B+Mathf.Sqrt(A*A+B*B-C*C))/(C-A));
        Theta2[i,1]=2*Mathf.Atan((-B-Mathf.Sqrt(A*A+B*B-C*C))/(C-A));
        print("Theta2["+i+",0]: "+Theta2[i,0]*180/Mathf.PI);
        print("Theta2["+i+",1]: "+Theta2[i,1]*180/Mathf.PI);
        }

        for(int i=0;i<2;i++)
        {
            for(int j=0;j<2;j++)
            {
            
                if(System.Math.Round(Mathf.Cos(Theta1[i]),2)==0)
                {
                    Theta3[i,j]=Mathf.Atan2(-(TargetSphereWrist[2,3]-A3*Mathf.Cos(Theta2[i,j]))/D4,-(A2+TargetSphereWrist[1,3]/Mathf.Sin(Theta1[i])+A3*Mathf.Sin(Theta2[i,j]))/D4)-Theta2[i,j];
                    print("Theta3["+i+","+j+"]: "+Theta3[i,j]*180/Mathf.PI);
                }
                else
                {
                    Theta3[i,j]=Mathf.Atan2(-(TargetSphereWrist[2,3]-A3*Mathf.Cos(Theta2[i,j]))/D4,-(A2-TargetSphereWrist[0,3]/Mathf.Cos(Theta1[i])+A3*Mathf.Sin(Theta2[i,j]))/D4)-Theta2[i,j];
                    print("Theta3["+i+","+j+"]: "+Theta3[i,j]*180/Mathf.PI);
                }
            
            }
        }

        for(int i=0;i<Theta2.GetLength(0);i++)
        {
            for(int j=0;j<Theta2.GetLength(1);j++)
            {
            while (Theta2[i,j]<-Mathf.PI)
            {
                Theta2[i,j]+=2*Mathf.PI;
            }
            while (Theta2[i,j]>Mathf.PI)
            {
                Theta2[i,j]-=2*Mathf.PI;
            }
            }
        }
        for(int i=0;i<Theta1.GetLength(0);i++)
        {
            while (Theta1[i]<-Mathf.PI)
            {
                Theta1[i]+=2*Mathf.PI;
            }
            while (Theta1[i]>Mathf.PI)
            {
                Theta1[i]-=2*Mathf.PI;
            }
        }
        for(int i=0;i<Theta3.GetLength(0);i++)
        {
            for(int j=0;j<Theta3.GetLength(1);j++)
            {
            while (Theta3[i,j]<-Mathf.PI)
            {
                Theta3[i,j]+=2*Mathf.PI;
            }
            while (Theta3[i,j]>Mathf.PI)
            {
                Theta3[i,j]-=2*Mathf.PI;
            }
            }
        }
        Matrix4x4 T30= new Matrix4x4();
        for(int i=0;i<2;i++)
        {
            for(int j=0;j<2;j++)
            {
                T30[0, 0] = Mathf.Sin(Theta2[i,j]+Theta3[i,j])*Mathf.Cos(Theta1[i]); T30[0, 1] = -Mathf.Sin(Theta2[i,j]+Theta3[i,j])*Mathf.Sin(Theta1[i]); T30[0, 2] = Mathf.Cos(Theta2[i,j]+Theta3[i,j]); T30[0, 3] = -A2*Mathf.Sin(Theta2[i,j]+Theta3[i,j])-A3*Mathf.Cos(Theta3[i,j]);
                // Row 1
                T30[1, 0] = Mathf.Cos(Theta2[i,j]+Theta3[i,j])*Mathf.Cos(Theta1[i]); T30[1, 1] = -Mathf.Cos(Theta2[i,j]+Theta3[i,j])*Mathf.Sin(Theta1[i]); T30[1, 2] = -Mathf.Sin(Theta2[i,j]+Theta3[i,j]); T30[1, 3] = A3*Mathf.Sin(Theta3[i,j])-A2*Mathf.Cos(Theta2[i,j]+Theta3[i,j])*Mathf.Cos(Theta1[i]);
                // Row 2
                T30[2, 0] = Mathf.Sin(Theta1[i]); T30[2, 1] = Mathf.Cos(Theta1[i]); T30[2, 2] = 0; T30[2, 3] = 0;
                // Row 3 (HoGogeneous coordinate - usually 0, 0, 0, 1)
                T30[3, 0] = 0; T30[3, 1] = 0; T30[3, 2] = 0; T30[3, 3] = 1;
                print("T30: "+T30);
                print("TargetSphereWrist: "+TargetSphereWrist);
                Matrix4x4 TargetRMatIn3=T30*TargetSphereWrist;
                print("TargetRMatIn3: "+TargetRMatIn3);
                Theta5[i,j,0]=Mathf.Acos(TargetRMatIn3[1,2]);
                Theta5[i,j,1]=-Mathf.Acos(TargetRMatIn3[1,2]);
                print("Theta5["+i+","+j+",0]: "+Theta5[i,j,0]*180/Mathf.PI);
                print("Theta5["+i+","+j+",1]: "+Theta5[i,j,1]*180/Mathf.PI);
                if(System.Math.Round(Mathf.Sin(Theta5[i,j,0]),2)==0)
                {
                    Theta4[i,j,0]=0;
                }
                else
                {
                    Theta4[i,j,0]=Mathf.Atan2(TargetRMatIn3[2,2]/-Mathf.Sin(Theta5[i,j,0]),TargetRMatIn3[0,2]/-Mathf.Sin(Theta5[i,j,0]));
                    print("Theta4["+i+","+j+",0]: "+Theta4[i,j,0]*180/Mathf.PI);
                }

                if(System.Math.Round(Mathf.Sin(Theta5[i,j,1]),2)==0)
                {
                    Theta4[i,j,1]=0;
                }
                else
                {
                    Theta4[i,j,1]=Mathf.Atan2(TargetRMatIn3[2,2]/-Mathf.Sin(Theta5[i,j,1]),TargetRMatIn3[0,2]/-Mathf.Sin(Theta5[i,j,1]));
                    print("Theta4["+i+","+j+",1]: "+Theta4[i,j,1]*180/Mathf.PI);
                }

                if(System.Math.Round(Mathf.Sin(Theta5[i,j,0]),2)==0)
                {
                    Theta6[i,j,0]=Mathf.Atan2(-TargetRMatIn3[2,0],TargetRMatIn3[2,1]);
                    print("Theta6["+i+","+j+",0]: "+Theta6[i,j,0]*180/Mathf.PI);
                }
                else
                {
                    Theta6[i,j,0]=Mathf.Atan2(TargetRMatIn3[1,1]/-Mathf.Sin(Theta5[i,j,0]),TargetRMatIn3[1,0]/-Mathf.Sin(Theta5[i,j,0]));
                    print("Theta6["+i+","+j+",0]: "+Theta6[i,j,0]*180/Mathf.PI);
                }

                if(System.Math.Round(Mathf.Sin(Theta5[i,j,1]),2)==0)
                {
                    Theta6[i,j,1]=Mathf.Atan2(-TargetRMatIn3[2,0],TargetRMatIn3[2,1]);
                    print("Theta6["+i+","+j+",1]: "+Theta6[i,j,1]*180/Mathf.PI);
                }
                else
                {
                    Theta6[i,j,1]=Mathf.Atan2(TargetRMatIn3[1,1]/-Mathf.Sin(Theta5[i,j,1]),TargetRMatIn3[1,0]/-Mathf.Sin(Theta5[i,j,1]));
                    print("Theta6["+i+","+j+",1]: "+Theta6[i,j,1]*180/Mathf.PI);
                }



            }
        }
        for(int i=0;i<Theta1.GetLength(0);i++)
        {
            Theta1[i]=TrueModulo(Theta1[i]-MinLimit[0],(2*Mathf.PI))+MinLimit[0];
        }
        for(int i=0;i<Theta2.GetLength(0);i++)
        {
            for(int j=0;j<Theta2.GetLength(1);j++)
            {
                Theta2[i,j]=TrueModulo(Theta2[i,j]-MinLimit[1],(2*Mathf.PI))+MinLimit[1];
            }
            
        }
        for(int i=0;i<Theta3.GetLength(0);i++)
        {
            for(int j=0;j<Theta3.GetLength(1);j++)
            {
                Theta3[i,j]=TrueModulo(Theta3[i,j]-MinLimit[2],(2*Mathf.PI))+MinLimit[2];
            }
            
        }

        for(int i=0;i<Theta4.GetLength(0);i++)
        {
            for(int j=0;j<Theta4.GetLength(1);j++)
            {
                for(int k=0;k<Theta4.GetLength(2);k++)
                {
                    Theta4[i,j,k]=TrueModulo(Theta4[i,j,k]-MinLimit[3],(2*Mathf.PI))+MinLimit[3];
                    Theta5[i,j,k]=TrueModulo(Theta5[i,j,k]-MinLimit[4],(2*Mathf.PI))+MinLimit[4];
                    Theta6[i,j,k]=TrueModulo(Theta6[i,j,k]-MinLimit[5],(2*Mathf.PI))+MinLimit[5];
                }
            }
            
        }




        print("Filtering Solutions...");
        List<float[]> Solutions = new List<float[]>();
        for(int i=0;i<Theta1.Length;i++)
        {
            for(int j=0;j<Theta2.GetLength(1);j++)
            {
                for(int k=0;k<Theta5.GetLength(2);k++)
                {
                    print("Trying Solution: Theta1["+i+"]: "+Theta1[i]*180/Mathf.PI+", Theta2["+i+","+j+"]: "+Theta2[i,j]*180/Mathf.PI+", Theta3["+i+","+j+"]: "+Theta3[i,j]*180/Mathf.PI+", Theta4["+i+","+j+","+k+"]: "+Theta4[i,j,k]*180/Mathf.PI+", Theta5["+i+","+j+","+k+"]: "+Theta5[i,j,k]*180/Mathf.PI+", Theta6["+i+","+j+","+k+"]: "+Theta6[i,j,k]*180/Mathf.PI);
                    if(!float.IsNaN(Theta1[i])&&!float.IsNaN(Theta2[i,j])&&!float.IsNaN(Theta3[i,j])&&!float.IsNaN(Theta4[i,j,k])&&!float.IsNaN(Theta5[i,j,k])&&!float.IsNaN(Theta6[i,j,k]))
                    {
                        Theta1[i]=Mathf.Round(Theta1[i]*100)/100;
                        Theta2[i,j]=Mathf.Round(Theta2[i,j]*100)/100;
                        Theta3[i,j]=Mathf.Round(Theta3[i,j]*100)/100;
                        Theta4[i,j,k]=Mathf.Round(Theta4[i,j,k]*100)/100;
                        Theta5[i,j,k]=Mathf.Round(Theta5[i,j,k]*100)/100;
                        Theta6[i,j,k]=Mathf.Round(Theta6[i,j,k]*100)/100;
                        print("Rounded Solution: Theta1["+i+"]: "+Theta1[i]+","+" Theta2["+i+","+j+"]: "+Theta2[i,j]+", "+" Theta3["+i+","+j+"]: "+Theta3[i,j]+", "+" Theta4["+i+","+j+","+k+"]: "+Theta4[i,j,k]+", "+" Theta5["+i+","+j+","+k+"]: "+Theta5[i,j,k]+", "+" Theta6["+i+","+j+","+k+"]: "+Theta6[i,j,k]);
                        if(Theta1[i]>=MinLimit[0]&&Theta1[i]<=MaxLimit[0]&&Theta2[i,j]>=MinLimit[1]&&Theta2[i,j]<=MaxLimit[1]&&Theta3[i,j]>=MinLimit[2]&&Theta3[i,j]<=MaxLimit[2]&&Theta4[i,j,k]>=MinLimit[3]&&Theta4[i,j,k]<=MaxLimit[3]&&Theta5[i,j,k]>=MinLimit[4]&&Theta5[i,j,k]<=MaxLimit[4]&&Theta6[i,j,k]>=MinLimit[5]&&Theta6[i,j,k]<=MaxLimit[5])
                        {
                            float[] solution= new float[6];
                            solution[0]=Theta1[i];
                            solution[1]=Theta2[i,j];
                            solution[2]=Theta3[i,j];
                            solution[3]=Theta4[i,j,k];
                            solution[4]=Theta5[i,j,k];
                            solution[5]=Theta6[i,j,k];
                            Solutions.Add(solution);
                        }else{
                            Solutions.Add(new float[]{float.NaN,float.NaN,float.NaN,float.NaN,float.NaN,float.NaN});
                        }
                        }else{
                            Solutions.Add(new float[]{float.NaN,float.NaN,float.NaN,float.NaN,float.NaN,float.NaN});
                        }
                    
                }
            }
        }
        
        
        return Solutions;


        
    }
    public Matrix4x4 FK(float[] Thetas)
    {
    
        Matrix4x4 G= new Matrix4x4();
        G[0, 0] = 1; G[0, 1] = 0; G[0, 2] = 0; G[0, 3] = 0;
        // Row 1
        G[1, 0] = 0; G[1, 1] = 1; G[1, 2] = 0; G[1, 3] = 0;
        // Row 2
        G[2, 0] = 0; G[2, 1] = 0; G[2, 2] = 1; G[2, 3] = .350f;
        // Row 3 (HoGogeneous coordinate - usually 0, 0, 0, 1)
        G[3, 0] = 0; G[3, 1] = 0; G[3, 2] = 0; G[3, 3] = 1;

        Matrix4x4 H= new Matrix4x4();
        H[0, 0] = 1; H[0, 1] = 0; H[0, 2] = 0; H[0, 3] = 0;
        // Row 1
        H[1, 0] = 0; H[1, 1] = 1; H[1, 2] = 0; H[1, 3] = 0;
        // Row 2
        H[2, 0] = 0; H[2, 1] = 0; H[2, 2] = 1; H[2, 3] = .075f;
        // Row 3 (HoHoHeneous coordinate - usually 0, 0, 0, 1)
        H[3, 0] = 0; H[3, 1] = 0; H[3, 2] = 0; H[3, 3] = 1;

        Matrix4x4 TMats1=new Matrix4x4();
        TMats1[0, 0] = Mathf.Cos(Thetas[0]); TMats1[0, 1] = Mathf.Sin(Thetas[0]); TMats1[0, 2] = 0; TMats1[0, 3] = 0;
        // Row 1
        TMats1[1, 0] = -Mathf.Sin(Thetas[0]); TMats1[1, 1] = Mathf.Cos(Thetas[0]); TMats1[1, 2] = 0; TMats1[1, 3] = 0;
        // Row 2
        TMats1[2, 0] = 0; TMats1[2, 1] = 0; TMats1[2, 2] = 1; TMats1[2, 3] = 0;
        // Row 3 (HoGogeneous coordinate - usually 0, 0, 0, 1)
        TMats1[3, 0] = 0; TMats1[3, 1] = 0; TMats1[3, 2] = 0; TMats1[3, 3] = 1;

        Matrix4x4 TMats2=new Matrix4x4();
        TMats2[0, 0] = Mathf.Cos(Thetas[1]-Mathf.PI/2); TMats2[0, 1] = -Mathf.Sin(Thetas[1]-Mathf.PI/2); TMats2[0, 2] = 0; TMats2[0, 3] = .100f;
        // Row 1
        TMats2[1, 0] = 0; TMats2[1, 1] = 0; TMats2[1, 2] = 1; TMats2[1, 3] = 0;
        // Row 2
        TMats2[2, 0] = -Mathf.Sin(Thetas[1]-Mathf.PI/2); TMats2[2, 1] = -Mathf.Cos(Thetas[1]-Mathf.PI/2); TMats2[2, 2] = 0; TMats2[2, 3] = 0;
        // Row 3 (HoGogeneous coordinate - usually 0, 0, 0, 1)
        TMats2[3, 0] = 0; TMats2[3, 1] = 0; TMats2[3, 2] = 0; TMats2[3, 3] = 1;

        Matrix4x4 TMats3=new Matrix4x4();
        TMats3[0, 0] = Mathf.Cos(Thetas[2]); TMats3[0, 1] = -Mathf.Sin(Thetas[2]); TMats3[0, 2] = 0; TMats3[0, 3] = .265f;
        // Row 1
        TMats3[1, 0] = Mathf.Sin(Thetas[2]); TMats3[1, 1] = Mathf.Cos(Thetas[2]); TMats3[1, 2] = 0    ; TMats3[1, 3] = 0;
        // Row 2
        TMats3[2, 0] = 0; TMats3[2, 1] = 0; TMats3[2, 2] = 1    ; TMats3[2, 3] = 0;
        // Row 3 (HoGogeneous coordinate - usually 0, 0, 0, 1)
        TMats3[3, 0] = 0; TMats3[3, 1] = 0; TMats3[3, 2] = 0    ; TMats3[3, 3] = 1;

        Matrix4x4 TMats4=new Matrix4x4();
        TMats4[0, 0] = Mathf.Cos(Thetas[3]); TMats4[0, 1] = Mathf.Sin(Thetas[3]); TMats4[0, 2] = 0; TMats4[0, 3] = 0;  
        // Row 1
        TMats4[1, 0] = 0; TMats4[1, 1] = 0; TMats4[1, 2] = 1; TMats4[1, 3] = .270f;
        // Row 2
        TMats4[2, 0] = Mathf.Sin(Thetas[3]); TMats4[2, 1] = -Mathf.Cos(Thetas[3]); TMats4[2, 2] = 0; TMats4[2, 3] = 0;
        // Row 3 (HoGogeneous coordinate - usually 0, 0, 0, 1)
        TMats4[3, 0] = 0    ; TMats4[3, 1] = 0; TMats4[3, 2] = 0; TMats4[3, 3] = 1;

        Matrix4x4 TMats5=new Matrix4x4();
        TMats5[0, 0] = Mathf.Cos(Thetas[4]); TMats5[0, 1] = -Mathf.Sin(Thetas[4]); TMats5[0, 2] = 0; TMats5[0, 3] = 0;  
        // Row 1
        TMats5[1, 0] = 0; TMats5[1, 1] = 0; TMats5[1, 2] = -1; TMats5[1, 3] = 0;
        // Row 2
        TMats5[2, 0] = Mathf.Sin(Thetas[4]); TMats5[2, 1] = Mathf.Cos(Thetas[4]); TMats5[2, 2] = 0; TMats5[2, 3] = 0;
        // Row 3 (HoGogeneous coordinate - usually 0, 0, 0, 1)
        TMats5[3, 0] = 0    ; TMats5[3, 1] = 0; TMats5[3, 2] = 0; TMats5[3, 3] = 1;

        Matrix4x4 TMats6=new Matrix4x4();
        TMats6[0, 0] = -Mathf.Cos(Thetas[5]); TMats6[0, 1] = -Mathf.Sin(Thetas[5]); TMats6[0, 2] = 0; TMats6[0, 3] = 0;
        // Row 1
        TMats6[1, 0] = 0; TMats6[1, 1] = 0; TMats6[1, 2] = 1; TMats6[1, 3] = 0;
        // Row 2
        TMats6[2, 0] = -Mathf.Sin(Thetas[5]); TMats6[2, 1] = Mathf.Cos(Thetas[5]); TMats6[2, 2] = 0; TMats6[2, 3] = 0;
        // Row 3 (HoGogeneous coordinate - usually 0, 0, 0, 1)
        TMats6[3, 0] = 0    ; TMats6[3, 1] = 0; TMats6[3, 2] =  0; TMats6[3, 3] = 1;



        print("qwer1: "+TMats1);
        print("qwer2: "+TMats1*TMats2);
        print("qwer3: "+TMats1*TMats2*TMats3);   
            print("qwer4: "+TMats1*TMats2*TMats3*TMats4);
                print("qwer5: "+TMats1*TMats2*TMats3*TMats4*TMats5);
                    print("qwer6: "+TMats1*TMats2*TMats3*TMats4*TMats5*TMats6);   

        Matrix4x4 T06=TMats1*TMats2*TMats3*TMats4*TMats5*TMats6;
        Matrix4x4 EndEffectorPose=G*T06*H;
        return EndEffectorPose;

    }
    public float TrueModulo(float a, float b)
    {
        // (a % b + b) % b ensures the result is always positive
        return (a % b + b) % b;
    }
    public void UnwrapPath(float[,] path)
    {
        int numJoints = path.GetLength(0); // 6
        int numSteps = path.GetLength(1);  // 100

        for (int step = 1; step < numSteps; step++)
        {
            for (int j = 0; j < numJoints; j++)
            {
                float prevAngle = path[j, step - 1];
                float currentAngle = path[j, step];

                // 1. Find the difference using the "Shortest Path" logic
                // Mathf.DeltaAngle returns a value between -180 and 180
                float diff = Mathf.DeltaAngle(prevAngle, currentAngle);

                // 2. Set the current angle to be Exactly (Previous + Shortest Difference)
                // This forces the angle to stay "near" the previous one, 
                // even if it has to go to 181 or -181 to do it.
                path[j, step] = (prevAngle + diff);

            }
            print("Unwrapped Angle at Step "+step+": "+path[0, step]+", "+path[1, step]+", "+path[2, step]+", "+path[3, step]+", "+path[4, step]+", "+path[5, step]);

        }
    }
    void OnGUI()
{
    if (showMessage)
    {
        // 1. Define the area (Center of screen, 300x100 pixels)
        Rect rect = new Rect(Screen.width / 2 - 150, Screen.height / 2 - 50, 300, 100);

        // 2. Start a simple Box area
        GUI.Box(rect, "IK Error Detected");

        // 3. Show the message inside the box
        GUI.Label(new Rect(rect.x + 10, rect.y + 30, 280, 40), errorMessage);

        // 4. Create a button to dismiss it
        if (GUI.Button(new Rect(rect.x + 100, rect.y + 70, 100, 20), "Close"))
        {
            showMessage = false;
        }
    }
}
public void TriggerError(string msg)
{
    errorMessage = msg;
    showMessage = true;
}

}