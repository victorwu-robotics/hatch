
"""
Hatch Platform Top-Level Architecture Bubble Diagram Generator
===============================================================

This script generates a bubble diagram of the Hatch (孵) robotics platform
architecture using matplotlib. It is organized into 7 horizontal layers
showing the full component hierarchy from Application down to Data & Assets.

Usage:
    python hatch_architecture_diagram.py

Output:
    hatch_top_level_architecture.pdf

You can modify bubble positions, sizes, colors, and text to adjust spacing
and appearance.
"""

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
import numpy as np

# Set font that supports Chinese
plt.rcParams['font.sans-serif'] = ['Noto Sans CJK SC', 'WenQuanYi Micro Hei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False  # Fix minus sign

# Disable automatic layout
plt.rcParams['figure.autolayout'] = False

# =============================================================================
# CONFIGURATION - YOUR ORIGINAL A4 SETTINGS PRESERVED
# =============================================================================

FIGURE_WIDTH = 8.27           # A4 width in inches
FIGURE_HEIGHT = 11.69         # A4 height in inches
DPI = 600                     # Output resolution

# Layer Y-positions (from bottom to top)
LAYER_Y = {
    'principles': 0.3,
    'data': 1.3,
    'event_flow': 2.75,
    'ui': 3.9,
    'viz': 5.7,
    'robot': 6.9,
    'core': 8.9,
    'app': 10.3,
    'title': 11.19,
}

# Bubble sizing defaults
BUBBLE_HEIGHT = 0.6
BUBBLE_WIDTH = 1.0
BUBBLE_PAD = 0.005
BUBBLE_ROUNDING = 0.15
BORDER_WIDTH = 1.2

# Font sizes
FONT_TITLE = 18
FONT_SUBTITLE = 10
FONT_LAYER_LABEL = 10
FONT_BUBBLE_MAIN = 9.5
FONT_BUBBLE_DETAIL = 8.0
FONT_ARROW_LABEL = 6.5
FONT_PRINCIPLES = 6.5

# Colors
COLORS = {
    'core': '#E8F4FD',
    'viz': '#D4EDDA',
    'robot': '#F8D7DA',
    'ui': '#FFF3CD',
    'comm': '#E2E3E5',
    'external': '#F5F5F5',
}

# =============================================================================
# HELPER FUNCTIONS
# =============================================================================

def draw_bubble(ax, x, y, w, h, text, color, fontsize=8, bold=False,
                border_color='black', border_width=1.2, pad=0.02, rounding=0.15):
    box = FancyBboxPatch((x - w/2, y - h/2), w, h,
                         boxstyle=f"round,pad={pad},rounding_size={rounding}",
                         facecolor=color, edgecolor=border_color, linewidth=border_width)
    ax.add_patch(box)
    weight = 'bold' if bold else 'normal'
    ax.text(x, y, text, fontsize=fontsize, ha='center', va='center',
            fontweight=weight, wrap=True, linespacing=1)
    return box


def draw_arrow(ax, x1, y1, x2, y2, color='black', style='->', lw=1,
               connectionstyle="arc3,rad=0", label=None, 
               label_offset=(0, 0.15), label_fraction=0.5):
    arrow = FancyArrowPatch((x1, y1), (x2, y2),
                            arrowstyle=style, color=color, lw=lw,
                            connectionstyle=connectionstyle,
                            mutation_scale=10)
    ax.add_patch(arrow)
    if label:
        # Calculate position along the arrow
        label_x = x1 + (x2 - x1) * label_fraction
        label_y = y1 + (y2 - y1) * label_fraction
        
        ax.text(label_x + label_offset[0], label_y + label_offset[1],
                label, fontsize=6.5, ha='center', va='center',
                color='darkblue', style='italic',
                bbox=dict(boxstyle='round,pad=0.1', facecolor='white',
                         edgecolor='none', alpha=0.8))
    return arrow


# =============================================================================
# MAIN DIAGRAM
# =============================================================================

def create_diagram():
    fig, ax = plt.subplots(1, 1, figsize=(FIGURE_WIDTH, FIGURE_HEIGHT),
                           dpi=DPI, constrained_layout=False)

    # Remove ALL padding - axes fill entire figure
    fig.subplots_adjust(left=0, right=1, bottom=0, top=1)
    
    # Remove axis margins
    ax.margins(0)

    ax.set_xlim(0, FIGURE_WIDTH)
    ax.set_ylim(0, FIGURE_HEIGHT)
    ax.axis('off')

    # ==================== TITLE ====================
    ax.text(4.135, LAYER_Y['title'], 'Hatch (孵) — Top-Level Platform Architecture',
            fontsize=FONT_TITLE, fontweight='bold', ha='center', va='center')
    ax.text(4.135, LAYER_Y['title'] - 0.25,
            'A Derived Architecture: Single Process, Single Memory Space, Event-Driven',
            fontsize=FONT_SUBTITLE, ha='center', va='center', style='italic', color='gray')

    # ==================== LAYER 1: APPLICATION ====================
    ax.text(0.4, LAYER_Y['app'] + 0.3, 'APPLICATION',
            fontsize=FONT_LAYER_LABEL, fontweight='bold', color='black')

    # MainWindow bubble
    mainwindow_x = 4.135
    mainwindow_y = LAYER_Y['app'] - 0.3
    mainwindow_w = 4
    mainwindow_h = 0.8
    
    draw_bubble(ax, mainwindow_x, mainwindow_y, mainwindow_w, mainwindow_h,
                'MainWindow\nCreates & owns all services, engine, robots, UI\nOrchestrates, does NOT hold business logic',
                COLORS['ui'], fontsize=FONT_BUBBLE_MAIN, bold=True,
                border_color='black', border_width=2, pad=BUBBLE_PAD)

    # MainWindow bottom edge for arrows
    mainwindow_bottom = mainwindow_y - mainwindow_h/2

    # ==================== LAYER 2: CORE SERVICES ====================
    ax.text(0.4, LAYER_Y['core'] + 0.3, 'CORE SERVICES',
            fontsize=FONT_LAYER_LABEL, fontweight='bold', color='#0C5460')

    # StateChannel
    statechannel_x = 1.5
    statechannel_y = LAYER_Y['core'] - 0.2
    statechannel_w = 1.6
    statechannel_h = 0.6
    
    draw_bubble(ax, statechannel_x, statechannel_y, statechannel_w, statechannel_h,
                'StateChannel\nPub/Sub events\nHistory & timestamps\nDecoupled\ncommunication',
                COLORS['core'], fontsize=FONT_BUBBLE_DETAIL, bold=True,
                border_color='#0C5460', pad=BUBBLE_PAD)

    # TransformRegistry
    transformregistry_x = 3.25
    transformregistry_y = LAYER_Y['core'] - 0.2
    transformregistry_w = 1.6
    transformregistry_h = 0.6
    
    draw_bubble(ax, transformregistry_x, transformregistry_y, transformregistry_w, transformregistry_h,
                'TransformRegistry\nAll spatial poses\nin one place\nLazy evaluation\nCache + callbacks',
                COLORS['core'], fontsize=FONT_BUBBLE_DETAIL, bold=True,
                border_color='#0C5460', pad=BUBBLE_PAD)

    # KinematicModel
    kinematicmodel_x = 5.0
    kinematicmodel_y = LAYER_Y['core'] - 0.2
    kinematicmodel_w = 1.6
    kinematicmodel_h = 0.6
    
    draw_bubble(ax, kinematicmodel_x, kinematicmodel_y, kinematicmodel_w, kinematicmodel_h,
                'KinematicModel\nURDF parsing\nForward kinematics\nTrue root detection',
                COLORS['core'], fontsize=FONT_BUBBLE_DETAIL, bold=True,
                border_color='#0C5460', pad=BUBBLE_PAD)

    # MeshLoader
    meshloader_x = 6.75
    meshloader_y = LAYER_Y['core'] - 0.2
    meshloader_w = 1.6
    meshloader_h = 0.6
    
    draw_bubble(ax, meshloader_x, meshloader_y, meshloader_w, meshloader_h,
                'MeshLoader\nLoad STL/OBJ meshes\nColor extraction',
                COLORS['core'], fontsize=FONT_BUBBLE_DETAIL, bold=True,
                border_color='#0C5460', pad=BUBBLE_PAD)

    # Arrows from MainWindow to Core (vertical down)
    draw_arrow(ax, mainwindow_x, mainwindow_bottom, 
               statechannel_x, statechannel_y + statechannel_h/2,
               color='gray', lw=1, label='creates', label_fraction=0.6)
    draw_arrow(ax, mainwindow_x, mainwindow_bottom, 
               transformregistry_x, transformregistry_y + transformregistry_h/2,
               color='gray', lw=1)
    draw_arrow(ax, mainwindow_x, mainwindow_bottom, 
               kinematicmodel_x, kinematicmodel_y + kinematicmodel_h/2,
               color='gray', lw=1)
    draw_arrow(ax, mainwindow_x, mainwindow_bottom, 
               meshloader_x, meshloader_y + meshloader_h/2,
               color='gray', lw=1)

    # ==================== LAYER 3: ROBOT SYSTEM ====================
    ax.text(0.4, LAYER_Y['robot'] + 0.9, 'ROBOT SYSTEM',
            fontsize=FONT_LAYER_LABEL, fontweight='bold', color='#721C24')

    # RobotManager
    robotmanager_x = 1.5
    robotmanager_y = LAYER_Y['robot'] + 0.4
    robotmanager_w = 1.6
    robotmanager_h = 0.6
    
    draw_bubble(ax, robotmanager_x, robotmanager_y, robotmanager_w, robotmanager_h,
                'RobotManager\nRobot lifecycle\nURDF loading\nMode switching',
                COLORS['robot'], fontsize=FONT_BUBBLE_DETAIL, bold=True,
                border_color='#721C24', pad=BUBBLE_PAD)

    # SimulatedRobot
    simulatedrobot_x = 3.25
    simulatedrobot_y = LAYER_Y['robot'] + 0.4
    simulatedrobot_w = 1.6
    simulatedrobot_h = 0.6
    
    draw_bubble(ax, simulatedrobot_x, simulatedrobot_y, simulatedrobot_w, simulatedrobot_h,
                'SimulatedRobot\nLocal IK solver\nPure Python simulation\nNo Qt',
                COLORS['robot'], fontsize=FONT_BUBBLE_DETAIL, bold=True,
                border_color='#721C24',pad=BUBBLE_PAD)

    # RealRobot
    realrobot_x = 5
    realrobot_y = LAYER_Y['robot'] + 0.4
    realrobot_w = 1.6
    realrobot_h = 0.6
    
    draw_bubble(ax, realrobot_x, realrobot_y, realrobot_w, realrobot_h,
                'RealRobot\nRTDE hardware bridge\nQt for signal handling',
                COLORS['robot'], fontsize=FONT_BUBBLE_DETAIL, bold=True,
                border_color='#721C24',pad=BUBBLE_PAD)

    # CommandHandler
    commandhandler_x = 6.75
    commandhandler_y = LAYER_Y['robot'] + 0.4
    commandhandler_w = 1.6
    commandhandler_h = 0.6
    
    draw_bubble(ax, commandhandler_x, commandhandler_y, commandhandler_w, commandhandler_h,
                'CommandHandler\nRoutes commands\nto active robot\nJOINT_COMMAND,\nCARTESIAN_COMMAND',
                COLORS['robot'], fontsize=FONT_BUBBLE_DETAIL, bold=True,
                border_color='#721C24', pad=BUBBLE_PAD)

    # StateHandler
    statehandler_x = 4.135
    statehandler_y = LAYER_Y['robot'] - 0.5
    statehandler_w = 5.5
    statehandler_h = 0.6
    
    draw_bubble(ax, statehandler_x, statehandler_y, statehandler_w, statehandler_h,
                'StateHandler\nSingle owner of model + registry updates\nReceives ROBOT_STATE → updates KinematicModel → updates TransformRegistry',
                COLORS['robot'], fontsize=FONT_BUBBLE_DETAIL, bold=True,
                border_color='#721C24', pad=BUBBLE_PAD)

    # Arrows from MainWindow to Robot (vertical down)
    draw_arrow(ax, mainwindow_x, mainwindow_bottom, 
               robotmanager_x, robotmanager_y + robotmanager_h/2,
               color='gray', lw=1, label='creates', label_fraction=0.8)
    draw_arrow(ax, mainwindow_x, mainwindow_bottom, 
               simulatedrobot_x, simulatedrobot_y + simulatedrobot_h/2,
               color='gray', lw=1)
    draw_arrow(ax, mainwindow_x, mainwindow_bottom, 
               realrobot_x, realrobot_y + realrobot_h/2,
               color='gray', lw=1)
    draw_arrow(ax, mainwindow_x, mainwindow_bottom, 
               commandhandler_x, commandhandler_y + commandhandler_h/2,
               color='gray', lw=1)

    # Arrows from Robot components to StateHandler (vertical down)
    draw_arrow(ax, robotmanager_x, robotmanager_y - robotmanager_h/2, 
               statehandler_x, statehandler_y + statehandler_h/2,
               color='#721C24', lw=1, label='loads URDF', label_fraction=0.2, label_offset=(0.01, -0.05))
    draw_arrow(ax, simulatedrobot_x, simulatedrobot_y - simulatedrobot_h/2, 
               statehandler_x, statehandler_y + statehandler_h/2,
               color='#721C24', lw=1)
    draw_arrow(ax, realrobot_x, realrobot_y - realrobot_h/2, 
               statehandler_x, statehandler_y + statehandler_h/2,
               color='#721C24', lw=1)
    draw_arrow(ax, commandhandler_x, commandhandler_y - commandhandler_h/2, 
               statehandler_x, statehandler_y + statehandler_h/2,
               color='#721C24', lw=1)

    # ==================== LAYER 4: VISUALIZATION ====================
    ax.text(0.4, LAYER_Y['viz'], 'VISUALIZATION',
            fontsize=FONT_LAYER_LABEL, fontweight='bold', color='#155724')

    # VisualizerEngine
    visualizerengine_x = 1.75
    visualizerengine_y = LAYER_Y['viz'] - 0.55
    visualizerengine_w = 2.5
    visualizerengine_h = 0.7
    
    draw_bubble(ax, visualizerengine_x, visualizerengine_y, visualizerengine_w, visualizerengine_h,
                'VisualizerEngine\nVTK render window\n60Hz timer\nLazy rendering (sleeps when idle)',
                COLORS['viz'], fontsize=FONT_BUBBLE_DETAIL, bold=True,
                border_color='#155724', pad=BUBBLE_PAD)

    # KinematicDisplay
    kinematicdisplay_x = 4.25
    kinematicdisplay_y = LAYER_Y['viz'] - 0.55
    kinematicdisplay_w = 2
    kinematicdisplay_h = 0.7
    
    draw_bubble(ax, kinematicdisplay_x, kinematicdisplay_y, kinematicdisplay_w, kinematicdisplay_h,
                'KinematicDisplay\nVTK actors for each link\nTransformRegistry callbacks\n_needs_render flag',
                COLORS['viz'], fontsize=FONT_BUBBLE_DETAIL, bold=True,
                border_color='#155724', pad=BUBBLE_PAD)

    # CameraManager
    cameramanager_x = 6.6
    cameramanager_y = LAYER_Y['viz'] - 0.55
    cameramanager_w = 2.25
    cameramanager_h = 0.7
    
    draw_bubble(ax, cameramanager_x, cameramanager_y, cameramanager_w, cameramanager_h,
                'CameraManager\nDiscovers cameras from URDF\nOne CameraPipeline per camera\n30 FPS each',
                COLORS['viz'], fontsize=FONT_BUBBLE_DETAIL, bold=True,
                border_color='#155724', pad=BUBBLE_PAD)

    # Arrows from MainWindow to Viz (vertical down)
    draw_arrow(ax, mainwindow_x, mainwindow_bottom, 
               visualizerengine_x, visualizerengine_y + visualizerengine_h/2,
               color='gray', lw=1, label='creates', label_fraction=0.95)
    draw_arrow(ax, mainwindow_x, mainwindow_bottom, 
               kinematicdisplay_x, kinematicdisplay_y + kinematicdisplay_h/2,
               color='gray', lw=1)
    draw_arrow(ax, mainwindow_x, mainwindow_bottom, 
               cameramanager_x, cameramanager_y + cameramanager_h/2,
               color='gray', lw=1)

    # Arrow from StateHandler to KinematicDisplay (diagonal down-right)
    draw_arrow(ax, statehandler_x, statehandler_y - statehandler_h/2, 
               kinematicdisplay_x, kinematicdisplay_y + kinematicdisplay_h/2,
               color='#155724', lw=1.5, label='transforms', label_offset=(-0.25,0.01))

    # ==================== LAYER 5: USER INTERFACE ====================
    ax.text(0.4, LAYER_Y['ui'] + 0.35, 'USER INTERFACE',
            fontsize=FONT_LAYER_LABEL, fontweight='bold', color='#856404')

    # UIBuilder
    uibuilder_x = 1.5
    uibuilder_y = LAYER_Y['ui'] - 0.15
    uibuilder_w = 1.6
    uibuilder_h = 0.6
    
    draw_bubble(ax, uibuilder_x, uibuilder_y, uibuilder_w, uibuilder_h,
                'UIBuilder\nMenus, toolbars, docks\nPanel construction',
                COLORS['ui'], fontsize=FONT_BUBBLE_DETAIL, bold=True,
                border_color='#856404', pad=BUBBLE_PAD)

    # MotionContainer
    motioncontainer_x = 3.25
    motioncontainer_y = LAYER_Y['ui'] - 0.15
    motioncontainer_w = 1.6
    motioncontainer_h = 0.6
    
    draw_bubble(ax, motioncontainer_x, motioncontainer_y, motioncontainer_w, motioncontainer_h,
                'MotionContainer\nJointControlPanel\nCartesianControlPanel\nRobotConnectionPanel',
                COLORS['ui'], fontsize=FONT_BUBBLE_DETAIL, bold=True,
                border_color='#856404', pad=BUBBLE_PAD)

    # CameraControlPanel
    cameracontrolpanel_x = 5
    cameracontrolpanel_y = LAYER_Y['ui'] - 0.15
    cameracontrolpanel_w = 1.6
    cameracontrolpanel_h = 0.6
    
    draw_bubble(ax, cameracontrolpanel_x, cameracontrolpanel_y, cameracontrolpanel_w, cameracontrolpanel_h,
                'CameraControlPanel\nSwitch between cameras\nStart/Stop/ROI/Visibility',
                COLORS['ui'], fontsize=FONT_BUBBLE_DETAIL,  bold=True,
                border_color='#856404', pad=BUBBLE_PAD)

    # JointFramePanel
    jointframepanel_x = 6.75
    jointframepanel_y = LAYER_Y['ui'] - 0.15
    jointframepanel_w = 1.6
    jointframepanel_h = 0.6
    
    draw_bubble(ax, jointframepanel_x, jointframepanel_y, jointframepanel_w, jointframepanel_h,
                'JointFramePanel\nShow/hide link frames\nScale & thickness',
                COLORS['ui'], fontsize=FONT_BUBBLE_DETAIL,  bold=True,
                border_color='#856404', pad=BUBBLE_PAD)

    # Arrows from MainWindow to UI (vertical down)
    draw_arrow(ax, mainwindow_x, mainwindow_bottom, 
               uibuilder_x, uibuilder_y + uibuilder_h/2,
               color='gray', lw=1, label='creates', label_fraction=0.95)
    draw_arrow(ax, mainwindow_x, mainwindow_bottom, 
               motioncontainer_x, motioncontainer_y + motioncontainer_h/2,
               color='gray', lw=1)
    draw_arrow(ax, mainwindow_x, mainwindow_bottom, 
               cameracontrolpanel_x, cameracontrolpanel_y + cameracontrolpanel_h/2,
               color='gray', lw=1)
    draw_arrow(ax, mainwindow_x, mainwindow_bottom, 
               jointframepanel_x, jointframepanel_y + jointframepanel_h/2,
               color='gray', lw=1)

    # ==================== LAYER 6: EVENT FLOW ====================
    ax.text(0.4, LAYER_Y['event_flow'] + 0.2, 'EVENT FLOW',
            fontsize=FONT_LAYER_LABEL, fontweight='bold', color='black')

    eventflow_x = 4.135
    eventflow_y = LAYER_Y['event_flow'] - 0.3
    eventflow_w = 5.5
    eventflow_h = 0.7
    
    draw_bubble(ax, eventflow_x, eventflow_y, eventflow_w, eventflow_h,
                'User moves slider → UI Panel publishes COMMAND → StateChannel distributes\n→ CommandHandler routes → Robot executes → publishes ROBOT_STATE\n→ StateHandler updates KinematicModel → TransformRegistry callbacks\n→ KinematicDisplay sets _needs_render → VisualizerEngine renders\n→ UI Panels receive ROBOT_STATE → update slider positions',
                COLORS['comm'], fontsize=FONT_BUBBLE_DETAIL, bold=True,
                border_color='black', pad=BUBBLE_PAD)

    # ==================== LAYER 7: DATA & ASSETS ====================
    ax.text(0.4, LAYER_Y['data'] + 0.3, 'DATA & ASSETS',
            fontsize=FONT_LAYER_LABEL, fontweight='bold', color='gray')

    # URDF / Xacro
    urdf_x = 1.5
    urdf_y = LAYER_Y['data'] - 0.2
    urdf_w = 1.6
    urdf_h = 0.6
    
    draw_bubble(ax, urdf_x, urdf_y, urdf_w, urdf_h,
                'URDF / Xacro\nScene description\nLinks, joints, meshes\nPackage:// paths',
                COLORS['external'], fontsize=FONT_BUBBLE_DETAIL, pad=BUBBLE_PAD)

    # URDFPreprocessor
    preprocessor_x = 3.25
    preprocessor_y = LAYER_Y['data'] - 0.2
    preprocessor_w = 1.6
    preprocessor_h = 0.6
    
    draw_bubble(ax, preprocessor_x, preprocessor_y, preprocessor_w, preprocessor_h,
                'URDFPreprocessor\nResolves includes\nSubstitutes variables\nExpands macros',
                COLORS['external'], fontsize=FONT_BUBBLE_DETAIL, pad=BUBBLE_PAD)

    # Assets
    assets_x = 5
    assets_y = LAYER_Y['data'] - 0.2
    assets_w = 1.6
    assets_h = 0.6
    
    draw_bubble(ax, assets_x, assets_y, assets_w, assets_h,
                'Assets\nScenes / Robots / Sensors\nUGV / Tools / Meshes',
                COLORS['external'], fontsize=FONT_BUBBLE_DETAIL, pad=BUBBLE_PAD)

    # Hardware
    hardware_x = 6.75
    hardware_y = LAYER_Y['data'] - 0.2
    hardware_w = 1.6
    hardware_h = 0.6
    
    draw_bubble(ax, hardware_x, hardware_y, hardware_w, hardware_h,
                'Hardware\nUR Robot (RTDE)\nCameras (USB/Network)',
                COLORS['external'], fontsize=FONT_BUBBLE_DETAIL, pad=BUBBLE_PAD)

    # Arrows between data components (horizontal)
    draw_arrow(ax, urdf_x + urdf_w/2, urdf_y, 
               preprocessor_x - preprocessor_w/2, preprocessor_y,
               color='gray', lw=1, label='feeds')
    draw_arrow(ax, preprocessor_x + preprocessor_w/2, preprocessor_y, 
               assets_x - assets_w/2, assets_y,
               color='gray', lw=1, label='loads')
    draw_arrow(ax, assets_x + assets_w/2, assets_y, 
               hardware_x - hardware_w/2, hardware_y,
               color='gray', lw=1, label='drives')

    # ==================== PRINCIPLES BANNER ====================
    ax.text(4.135, LAYER_Y['principles'],
            'Principles: #0 Individuals Before Groups  |  #1 Single Process  |  #2 Event-Driven  |  #3 Visualizer as Mind-Prying Tool  |  #4 Everything in URDF  |\n  #5 Space = TransformRegistry  |  #6 Time = StateChannel  |  #7 Movements as Models  |  #8 Pure Python  |  #9 UI Separate from Services  |  #10 One Robot Per Session',
            fontsize=FONT_PRINCIPLES, ha='center', va='center', style='italic', color='gray',
            bbox=dict(boxstyle='round,pad=0.3', facecolor='white', edgecolor='gray', alpha=0.8))

    # Save and show
    plt.savefig('hatch_top_level_architecture.pdf', format='pdf',
                facecolor='white', edgecolor='none',
                bbox_inches=None, pad_inches=0) # No bbox padding
    print("Diagram saved to hatch_top_level_architecture.pdf")


if __name__ == "__main__":
    create_diagram()
