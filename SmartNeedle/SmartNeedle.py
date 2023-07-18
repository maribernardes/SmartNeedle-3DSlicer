import logging
import os

import vtk, qt, ctk, slicer
from slicer.ScriptedLoadableModule import *
from slicer.util import VTKObservationMixin
from PythonQt import QtCore, QtGui

import SimpleITK as sitk
import sitkUtils
import numpy as np
import CurveMaker
from skimage.restoration import unwrap_phase


class SmartNeedle(ScriptedLoadableModule):

  def __init__(self, parent):
    ScriptedLoadableModule.__init__(self, parent)
    self.parent.title = "SmartNeedle"
    self.parent.categories = ["IGT"] 
    self.parent.dependencies = ["ZFrameRegistration", "OpenIGTLinkIF", "CurveMaker"]  # TODO: add here list of module names that this module requires
    self.parent.contributors = ["Mariana Bernardes (BWH), Lisa Mareschal (BWH), Pedro Moreira (BWH), Junichi Tokuda (BWH)"] 
    self.parent.helpText = """ This module is used to interact with the ROS2 packages for SmartNeedle shape sensing. Uses ZFrameRegistration module for initialization of the ZTransform, and OpenIGTLink to communicate with ROS2OpenIGTLinkBridge """
    # TODO: replace with organization, grant and thanks
    self.parent.acknowledgementText = """ """

    # Additional initialization step after application startup is complete
    # TODO: include sample data and testing routines
    # slicer.app.connect("startupCompleted()", registerSampleData)

################################################################################################################################################
# Widget Class
################################################################################################################################################

class SmartNeedleWidget(ScriptedLoadableModuleWidget, VTKObservationMixin):
  """Uses ScriptedLoadableModuleWidget base class, available at:
  https://github.com/Slicer/Slicer/blob/main/Base/Python/slicer/ScriptedLoadableModule.py
  """
  # Called when the user opens the module the first time and the widget is initialized.
  def __init__(self, parent=None):
    ScriptedLoadableModuleWidget.__init__(self, parent)
    VTKObservationMixin.__init__(self)  # needed for parameter node observation
    self.logic = None
    self._parameterNode = None
    self._updatingGUIFromParameterNode = False

  ### Widget setup ###################################################################
  def setup(self):
    ScriptedLoadableModuleWidget.setup(self)
    print('Widget Setup')
    ####################################
    ##                                ##
    ## UI Components                  ##
    ##                                ##
    ####################################

    ## Server collapsible button            
    ####################################
    connectionCollapsibleButton = ctk.ctkCollapsibleButton()
    connectionCollapsibleButton.text = 'Robot connection'
    self.layout.addWidget(connectionCollapsibleButton)
    connectionLayout = qt.QFormLayout(connectionCollapsibleButton)

    # ZTransform matrix
    self.zTransformSelector = slicer.qMRMLNodeComboBox()
    self.zTransformSelector.nodeTypes = ['vtkMRMLLinearTransformNode']
    self.zTransformSelector.selectNodeUponCreation = True
    self.zTransformSelector.addEnabled = True
    self.zTransformSelector.removeEnabled = True
    self.zTransformSelector.noneEnabled = True
    self.zTransformSelector.showHidden = False
    self.zTransformSelector.showChildNodeTypes = False
    self.zTransformSelector.setMRMLScene(slicer.mrmlScene)
    self.zTransformSelector.setToolTip('Select the ZFrame Transform')
    connectionLayout.addRow('ZTransform:', self.zTransformSelector)
   
    # Port (No need for IP - Server)
    self.portTextbox = qt.QLineEdit('18944')
    self.portTextbox.setReadOnly(False)
    self.portTextbox.setMaximumWidth(250)
    connectionLayout.addRow('Port:', self.portTextbox)
     
    # Start/Stop buttons 
    buttonsHBoxLayout = qt.QHBoxLayout()    
    self.startButton = qt.QPushButton('Start server')
    self.startButton.toolTip = 'Start OpenIGTLink server'
    self.startButton.enabled = False
    buttonsHBoxLayout.addWidget(self.startButton)
    self.stopButton = qt.QPushButton('Stop server')
    self.stopButton.toolTip = 'Stop the needle tracking'
    self.stopButton.enabled = False    
    buttonsHBoxLayout.addWidget(self.stopButton)
    connectionLayout.addRow('', buttonsHBoxLayout)
    
    # Server status
    self.statusLabel = qt.QLineEdit('<IGTLink Server Status>')
    self.statusLabel.setReadOnly(True)
    connectionLayout.addRow('', self.statusLabel)
    
    ## Planning collapsible button                 
    ####################################
    
    planningCollapsibleButton = ctk.ctkCollapsibleButton()
    planningCollapsibleButton.text = 'Planning'
    self.layout.addWidget(planningCollapsibleButton)   
    planningFormLayout = qt.QVBoxLayout(planningCollapsibleButton)
    
    # Points selection
    markupsLayout = qt.QFormLayout()
    planningFormLayout.addLayout(markupsLayout)
    self.pointListSelector = slicer.qSlicerSimpleMarkupsWidget()    
    self.pointListSelector.setMRMLScene(slicer.mrmlScene)
    self.pointListSelector.markupsPlaceWidget().setPlaceMultipleMarkups(slicer.qSlicerMarkupsPlaceWidget().ForcePlaceMultipleMarkups)
    self.pointListSelector.defaultNodeColor = qt.QColor(170,0,0)
    self.pointListSelector.setNodeSelectorVisible(False)
    self.pointListSelector.tableWidget().show()
    self.pointListSelector.toolTip = 'Select 2 points: ENTRY and TARGET'
    markupsLayout.addRow('Planned points:', self.pointListSelector)
    
    # SendPlan button 
    self.sendButton = qt.QPushButton('Send planned points')
    self.sendButton.toolTip = 'Send planned points to OpenIGTLink server'
    self.sendButton.enabled = False
    markupsLayout.addRow('', self.sendButton)

    ## Shape sensing collapsible button                
    ####################################
    
    sensingCollapsibleButton = ctk.ctkCollapsibleButton()
    sensingCollapsibleButton.text = 'Needle Shape Sensing'    
    self.layout.addWidget(sensingCollapsibleButton)
    sensingFormLayout = qt.QFormLayout(sensingCollapsibleButton)
        
    # Print current Shape Sensing header  
    self.timeStampTextbox = qt.QLineEdit('-- : -- : --.----')
    self.timeStampTextbox.setReadOnly(True)
    self.timeStampTextbox.setStyleSheet('background-color: transparent; border: no border;')
    self.timeStampTextbox.toolTip = 'Timestamp from last shape measurement'
    sensingFormLayout.addRow('Needle shape timestamp:', self.timeStampTextbox)

    self.numberPointsTextbox = qt.QLineEdit('')
    self.numberPointsTextbox.setReadOnly(True)
    self.numberPointsTextbox.setStyleSheet('background-color: transparent; border: no border;')
    self.numberPointsTextbox.toolTip = 'Number of points in needle shape'
    sensingFormLayout.addRow('Number points:', self.numberPointsTextbox)
    
    # Print current needle tip                   
    self.needleTipTextbox = qt.QLineEdit('(---, ---, ---)')
    self.needleTipTextbox.setReadOnly(True)
    self.needleTipTextbox.setStyleSheet('background-color: transparent; border: no border;')
    self.needleTipTextbox.toolTip = 'Needle tip current position'
    sensingFormLayout.addRow('Tip coordinates (RAS):', self.needleTipTextbox)
    self.layout.addStretch(1)
    
    ####################################
    ##                                ##
    ## UI Behavior                    ##
    ##                                ##
    ####################################

    # Initialize module logic
    self.logic = SmartNeedleLogic()
        
    # These connections ensure that we update parameter node when scene is closed
    self.addObserver(slicer.mrmlScene, slicer.mrmlScene.StartCloseEvent, self.onSceneStartClose)
    self.addObserver(slicer.mrmlScene, slicer.mrmlScene.EndCloseEvent, self.onSceneEndClose)
    # self.addObserver(slicer.mrmlScene, slicer.vtkMRMLScene.NodeAddedEvent, self.nodeAddedCallback)

    # These connections ensure that we update server status when IGTLConnectorNode changes
    self.addObserver(self.logic.serverNode, slicer.vtkMRMLIGTLConnectorNode.ActivatedEvent, self.onConnectionStatusChange)
    self.addObserver(self.logic.serverNode, slicer.vtkMRMLIGTLConnectorNode.DeactivatedEvent, self.onConnectionStatusChange)
    self.addObserver(self.logic.serverNode, slicer.vtkMRMLIGTLConnectorNode.ConnectedEvent, self.onConnectionStatusChange)
    self.addObserver(self.logic.serverNode, slicer.vtkMRMLIGTLConnectorNode.DisconnectedEvent, self.onConnectionStatusChange)
    
    # This connection updates the needle shape when a new message comes
    self.addObserver(self.logic.needleShapeHeaderNode, slicer.vtkMRMLTextNode.TextModifiedEvent, self.onNeedleShapeChange)
        
    # These connections ensure that whenever user changes some settings on the GUI, that is saved in the MRML scene
    # (in the selected parameter node).
    self.portTextbox.connect('textChanged', self.updateParameterNodeFromGUI)
    self.zTransformSelector.connect('currentNodeChanged(vtkMRMLNode*)', self.updateParameterNodeFromGUI)
    
    # Connect Qt widgets to event calls
    self.zTransformSelector.connect('currentNodeChanged(vtkMRMLNode*)', self.onZTranformChange)
    self.startButton.connect('clicked(bool)', self.startServer)
    self.stopButton.connect('clicked(bool)', self.stopServer)
    self.sendButton.connect('clicked(bool)', self.sendPoints)
    self.pointListSelector.connect('updateFinished()', self.onPointListChanged)

    # Make sure parameter node is initialized (needed for module reload)
    self.initializeParameterNode()
    
    # Initialize widget variables and updateGUI
    self.updateGUI()
    self.pointListSelector.setCurrentNode(self.logic.pointListNode)
    self.pointListSelector.markupsPlaceWidget().placeButton().setChecked(False)

  ### Widget functions ###################################################################
  # Called when the application closes and the module widget is destroyed.
  def cleanup(self):
    self.removeObservers()

  # Called each time the user opens this module.
  # Make sure parameter node exists and observed
  def enter(self):
    self.initializeParameterNode() 

  # Called each time the user opens a different module.
  # Do not react to parameter node changes (GUI will be updated when the user enters into the module)
  def exit(self):
    self.removeObserver(self._parameterNode, vtk.vtkCommand.ModifiedEvent, self.updateGUIFromParameterNode)

  # Called just before the scene is closed.
  # Parameter node will be reset, do not use it anymore
  def onSceneStartClose(self, caller, event):
    self.logic.closeConnection()    
    self.setParameterNode(None)

  # Called just after the scene is closed.
  # If this module is shown while the scene is closed then recreate a new parameter node immediately
  def onSceneEndClose(self, caller, event):
    if self.parent.isEntered:
      self.initializeParameterNode()
        
  # Ensure parameter node exists and observed
  # Parameter node stores all user choices in parameter values, node selections, etc.
  # so that when the scene is saved and reloaded, these settings are restored.
  def initializeParameterNode(self):
    print('initializeParameterNode')
    # Load default parameters in logic module
    self.setParameterNode(self.logic.getParameterNode())
    # Select default input nodes if nothing is selected yet to save a few clicks for the user
    if not self._parameterNode.GetNodeReference('ZTransform'):
      # Find first selectable transform
      zTransformNode = next((node for node in slicer.util.getNodesByClass('vtkMRMLLinearTransformNode') if node.GetSelectable()==1), None)
      if zTransformNode:
        self._parameterNode.SetNodeReferenceID('ZTransform', zTransformNode.GetID())
            
  # Set and observe parameter node.
  # Observation is needed because when the parameter node is changed then the GUI must be updated immediately.
  def setParameterNode(self, inputParameterNode):
    print('setParameterNode')
    if inputParameterNode:
      self.logic.setDefaultParameters(inputParameterNode)
    # Unobserve previously selected parameter node and add an observer to the newly selected.
    # Changes of parameter node are observed so that whenever parameters are changed by a script or any other module
    # those are reflected immediately in the GUI.
    if self._parameterNode is not None and self.hasObserver(self._parameterNode, vtk.vtkCommand.ModifiedEvent, self.updateGUIFromParameterNode):
        self.removeObserver(self._parameterNode, vtk.vtkCommand.ModifiedEvent, self.updateGUIFromParameterNode)
    self._parameterNode = inputParameterNode
    if self._parameterNode is not None:
        self.addObserver(self._parameterNode, vtk.vtkCommand.ModifiedEvent, self.updateGUIFromParameterNode)
    # Initial GUI update
    self.updateGUIFromParameterNode()

  # This method is called whenever parameter node is changed.
  # The module GUI is updated to show the current state of the parameter node.
  def updateGUIFromParameterNode(self, caller=None, event=None):
    print('updateGUIFromParameterNode')
    if self._parameterNode is None or self._updatingGUIFromParameterNode:
      return
    # Make sure GUI changes do not call updateParameterNodeFromGUI (it could cause infinite loop)
    self._updatingGUIFromParameterNode = True
    # Update node selectors and input boxes and sliders
    self.portTextbox.setText(self._parameterNode.GetParameter('Port'))
    self.zTransformSelector.setCurrentNode(self._parameterNode.GetNodeReference('ZTransform'))
    self.pointListSelector.setCurrentNode(self._parameterNode.GetNodeReference('Planning'))
    self.updateGUI()
    # All the GUI updates are done
    self._updatingGUIFromParameterNode = False

  # This method is called when the user makes any change in the GUI.
  # The changes are saved into the parameter node (so that they are restored when the scene is saved and loaded).
  def updateParameterNodeFromGUI(self, caller=None, event=None):
    print('updateParameterNodeFromGUI')
    if self._parameterNode is None or self._updatingGUIFromParameterNode:
      return
    # Modify all properties in a single batch
    wasModified = self._parameterNode.StartModify()  
    # Update paramenters_nodes
    self._parameterNode.SetParameter('Port', self.portTextbox.text.strip())
    self._parameterNode.SetNodeReferenceID('ZTransform', self.zTransformSelector.currentNodeID)
    # All paramenter_nodes updates are done
    self._parameterNode.EndModify(wasModified)
  
  # Called when a new zTransform is selected
  def onZTranformChange(self):
    print('Updated ZTransform')
    
  # Called when the Point List is updated
  def onPointListChanged(self):    
    self.logic.addPlanningPoint(self.pointListSelector.currentNode())
    self.updateGUI()
  
  # Update the connection statusLabel  
  def onConnectionStatusChange(self, caller=None, event=None):
    self.updateGUI()

# Callback function to check if the node exists and call the update function
  def onNeedleShapeChange(self, caller=None, event=None):
    # Check if Registration transform is selected
    if (self.zTransformSelector.currentNode() is not None):
      # Update header info
      (timestamp, package_number, num_points, frame_id) = self.logic.getCurrentHeader()    
      print('*** Received Needle Shape ***')
      print('Timestamp:', timestamp)
      print('Package Number:', package_number)
      print('Number of Points:', num_points)
      print('Frame ID:', frame_id)
      zTransformNode = self.zTransformSelector.currentNode()
      self.logic.updateNeedleShapeCurve(zTransformNode)
    
  # Update GUI buttons, markupWidget and connection statusLabel
  def updateGUI(self):
    # Check status
    connectionStatus = self.logic.getConnectionStatus()
    serverActive = (connectionStatus == slicer.vtkMRMLIGTLConnectorNode.StateWaitConnection) or (connectionStatus == slicer.vtkMRMLIGTLConnectorNode.StateConnected)
    serverDefined = (self.portTextbox.text.strip() != '')
    zFrameSelected = (self.zTransformSelector.currentNode() is not None)
    pointsSelected = (self.logic.getNumberOfPoints(self.pointListSelector.currentNode()) == 2)
    # Update buttons accordingly
    self.startButton.enabled = serverDefined and not serverActive
    self.stopButton.enabled = serverActive
    self.sendButton.enabled = pointsSelected and zFrameSelected #and serverActive
    # Update markup widget accordingly
    self.pointListSelector.placeActive(not pointsSelected)
    # Update connection status label
    if (connectionStatus == slicer.vtkMRMLIGTLConnectorNode.StateOff):               # 0 - OFF
      self.statusLabel.setStyleSheet('background-color: pink; border: 1px solid black;')
      self.statusLabel.setText('Server inactive')
    elif (connectionStatus == slicer.vtkMRMLIGTLConnectorNode.StateWaitConnection):  # 1 - WAIT
      self.statusLabel.setStyleSheet('background-color: lightyellow; border: 1px solid black;')
      self.statusLabel.setText('Waiting for client')
    elif (connectionStatus == slicer.vtkMRMLIGTLConnectorNode.StateConnected):       # 2 - ON
      self.statusLabel.setStyleSheet('background-color: lightgreen; border: 1px solid black;')
      self.statusLabel.setText('Connected to client... Ready to send/receive!')
    else:                                                                                 # ANY OTHER STATE
      self.statusLabel.setStyleSheet('background-color: pink; border: 1px solid black;')
      self.statusLabel.setText('Error with OpenIGTLink server node')

  def startServer(self):
    print('UI: startServer()')
    # Get Server Port value
    serverPort = self.portTextbox.text.strip()
    # Start server
    self.logic.activateServer(serverPort)
  
  def stopServer(self):
    print('UI: stopTracking()')
    # Stop server
    self.logic.deactivateServer()
    
  def sendPoints(self):
    print('UI: sendPoints()')
    # Get current transform node
    zTransformNode = self.zTransformSelector.currentNode()
    pointListNode = self.pointListSelector.currentNode()
    self.logic.sendPlannedPoints(pointListNode, zTransformNode)
    
################################################################################################################################################
# Logic Class
################################################################################################################################################

class SmartNeedleLogic(ScriptedLoadableModuleLogic):

  def __init__(self):
    ScriptedLoadableModuleLogic.__init__(self)
    self.cliParamNode = None
    print('Logic: __init__')
    self.curveMaker = CurveMaker.CurveMakerLogic()
    self.initializeNodes()
    
  ### Logic setup ###################################################################
  def initializeNodes(self):  
    # Create OpenIGTLink Server Nodes and initialize logic variables
    try:
      self.serverNode = slicer.util.getNode('IGTLSmartNeedleServer')
      self.serverNode.Stop()
    except:
      self.serverNode = slicer.vtkMRMLIGTLConnectorNode()
      slicer.mrmlScene.AddNode(self.serverNode)
      self.serverNode.SetName('IGTLSmartNeedleServer')
    # Create Transform nodes
    try:
      self.worldToZFrameTransformNode = slicer.util.getNode('WorldToZFrameTransform')
    except:
      self.worldToZFrameTransformNode = slicer.vtkMRMLTransformNode()
      slicer.mrmlScene.AddNode(self.worldToZFrameTransformNode)
      self.worldToZFrameTransformNode.SetName('WorldToZFrameTransform')
    # Create PointList node for planning
    try:
      self.pointListNode = slicer.util.getNode('Planning')
    except:
      self.pointListNode = slicer.vtkMRMLMarkupsFiducialNode()
      slicer.mrmlScene.AddNode(self.pointListNode)
      self.pointListNode.SetName('Planning')
    # Create PointListZ node for planning
    try:
      self.zFramePointListNode = slicer.util.getNode('PlanningZ')
    except:
      self.zFramePointListNode = slicer.vtkMRMLMarkupsFiducialNode()
      slicer.mrmlScene.AddNode(self.zFramePointListNode)
      self.zFramePointListNode.SetName('PlanningZ')
    # Create NeedleShape message header
    try:
      self.needleShapeHeaderNode = slicer.util.getNode('NeedleShapeHeader')
    except:
      self.needleShapeHeaderNode = slicer.vtkMRMLTextNode()
      slicer.mrmlScene.AddNode(self.needleShapeHeaderNode)
      self.needleShapeHeaderNode.SetName('NeedleShapeHeader')
    # Create the NeedleShape points node
    try:
      self.needleShapePointsNode = slicer.util.getNode('NeedleShape')
    except:
      self.needleShapePointsNode = slicer.vtkMRMLMarkupsFiducialNode()
      slicer.mrmlScene.AddNode(self.needleShapePointsNode)
      self.needleShapePointsNode.SetName('NeedleShape')
    # Create the NeedleShapeZ points node
    try:
      self.zFrameNeedleShapePointsNode = slicer.util.getNode('NeedleShapeZ')
    except:
      self.zFrameNeedleShapePointsNode = slicer.vtkMRMLMarkupsFiducialNode()
      slicer.mrmlScene.AddNode(self.zFrameNeedleShapePointsNode)
      self.zFrameNeedleShapePointsNode.SetName('NeedleShapeZ')
    # Create the Needle Model node
    try:
      self.needleModelNode = slicer.util.getNode('NeedleModel')
    except:
      self.needleModelNode = slicer.vtkMRMLModelNode()
      slicer.mrmlScene.AddNode(self.needleModelNode)
      self.needleModelNode.SetName('NeedleModel')   
      self.needleModelNode.SetDisplayVisibility(True)
      displayNode = self.needleModelNode.GetDisplayNode()
      if displayNode is None:
          displayNode = slicer.vtkMRMLModelDisplayNode()
          # displayNode = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLModelDisplayNode')
          self.needleModelNode.SetAndObserveDisplayNodeID(displayNode.GetID())
      # displayNode.SetColor(0.678, 0.847, 0.902)  # Light blue color
      # displayNode.Modified()
      
    # Get CurveMaker module logic
    self.curveMaker.SourceNode = self.needleShapePointsNode
    self.curveMaker.DestinationNode = self.needleModelNode
    self.curveMaker.AutomaticUpdate = True
    self.curveMaker.TubeRadius = 1.5
    self.curveMaker.ModelColor = [0.678, 0.847, 0.902]   

  ### Logic functions ###################################################################
  # Initialize parameter node with default settings
  def setDefaultParameters(self, parameterNode):
    # self.initialize()
    if not parameterNode.GetParameter('Port'):
      parameterNode.SetParameter('Port', '18944')
    if not parameterNode.GetNodeReference('Planning'):
      parameterNode.SetNodeReferenceID('Planning', self.pointListNode.GetID())
      
  # Restart server connection
  def activateServer(self, serverPort):
    self.serverNode.Stop()
    self.serverNode.SetType(slicer.vtkMRMLIGTLConnectorNode.TypeServer)
    self.serverNode.SetServerPort(int(serverPort))
    # Register input and output nodes
    if self.needleShapeHeaderNode is not None:
      self.serverNode.RegisterIncomingMRMLNode(self.needleShapeHeaderNode)
    if self.zFrameNeedleShapePointsNode is not None:
      self.serverNode.RegisterIncomingMRMLNode(self.zFrameNeedleShapePointsNode)
    if self.zFramePointListNode is not None:
      self.serverNode.RegisterOutgoingMRMLNode(self.zFramePointListNode)
    self.serverNode.Start()
          
  def deactivateServer(self):
    self.serverNode.Stop()

  # Close client connection
  # Remove OpenIGTLink Client Node from mrmlScene
  def closeConnection(self):
    if self.serverNode is not None:
      try:
        self.serverNode.Stop()
        slicer.mrmlScene.RemoveNode(self.serverNode)
        return True
      except:
        print('Error closing OpenIGTLink client node')
        return False
      
  def getConnectionStatus(self):
    return self.serverNode.GetState()
  
  def getNumberOfPoints(self, pointListNode):
    if pointListNode is not None:
      return pointListNode.GetNumberOfDefinedControlPoints()
    else: 
      return 0
  
  def addPlanningPoint(self, pointListNode):
    if pointListNode is not None:
      N = pointListNode.GetNumberOfControlPoints()
      if N>=1:
        pointListNode.SetNthControlPointLabel(0, 'ENTRY')
      if N>=2:
        pointListNode.SetNthControlPointLabel(1, 'TARGET')
      if pointListNode.GetNumberOfDefinedControlPoints()>=2:
        pointListNode.SetFixedNumberOfControlPoints(2)
  
  # Extract information from NeedleShapeHeader STRING message
  def getCurrentHeader(self):
    headerText = self.needleShapeHeaderNode.GetText()
    # Split the text using the ';' delimiter
    parts = headerText.split(';')
    # Extract the individual values and assign them to separate variables
    timestamp = parts[0]
    package_number = int(parts[1])
    num_points = int(parts[2])
    frame_id = parts[3]
    return (timestamp, package_number, num_points, frame_id) 
  
  # Send planned points through IGTLink server
  def sendPlannedPoints(self, pointListNode, zTransformNode):
    if zTransformNode is None:
      print('Select a ZTransform first')
      return False
    # Get world to ZFrame transformations
    worldToZFrame = vtk.vtkMatrix4x4()
    zTransformNode.GetMatrixTransformFromWorld(worldToZFrame)
    # Set it to worldToZFrameNode
    self.worldToZFrameTransformNode.SetMatrixTransformToParent(worldToZFrame)
    # Make a copy of ZFrame Points coordinates
    self.zFramePointListNode.CopyContent(pointListNode)
    # Apply zTransform to points
    self.zFramePointListNode.SetAndObserveTransformNodeID(self.worldToZFrameTransformNode.GetID())
    self.zFramePointListNode.HardenTransform()
    # Push to IGTLink Server
    self.serverNode.RegisterOutgoingMRMLNode(self.zFramePointListNode)
    self.serverNode.PushNode(self.zFramePointListNode)
    return True

  # Update needle model from NeedleShape POINT_ARRAY message
  def updateNeedleShapeCurve(self, zTransformNode):
    if zTransformNode is None:
      print('Select a ZTransform first')
      return False
    if self.zFrameNeedleShapePointsNode.GetNumberOfControlPoints() >= 2:
      # Make a copy of NeedleShape Z points 
      self.needleShapePointsNode.CopyContent(self.zFrameNeedleShapePointsNode)
      # Apply zFrame to World transform to needle shape points
      self.needleShapePointsNode.SetAndObserveTransformNodeID(zTransformNode.GetID())
      self.needleShapePointsNode.HardenTransform()
      self.curveMaker.updateCurve()
      return True