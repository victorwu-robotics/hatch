
"""
Hatch Platform Top-Level Architecture Bubble Diagram Generator
===============================================================

This script generates a bubble diagram of the Hatch (孵) robotics platform
architecture using matplotlib. It is organized into 7 horizontal layers
showing the full component hierarchy from Application down to Data & Assets.

Usage:
    python hatch_architecture_diagram.py

Output:
    hatch_top_level_architecture.png

You can modify bubble positions, sizes, colors, and text to adjust spacing
and appearance.
"""

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
import numpy as np

# =============================================================================
# CONFIGURATION - Adjust these values to control spacing and sizing
# =============================================================================

FIGURE_WIDTH = 12          # Total figure width in inches
FIGURE_HEIGHT = 22         # Total figure height in inches
DPI = 150                  # Output resolution

# Layer Y-positions (from bottom to top). Increase gaps between layers for more space.
LAYER_Y = {
    'principles': 0.6,
    'data': 1.8,
    'event_flow': 4.7,
    'ui': 7.5,
    'viz': 10.0,
    'robot': 13.3,
    'core': 16.2,
    'app': 19.0,
    'title': 21.0,
}

# Bubble sizing defaults
BUBBLE_HEIGHT = 0.6        # Default bubble height
BUBBLE_WIDTH = 1.0         # Default bubble width
BUBBLE_PAD = 0.005          # Internal padding (text to bubble edge)
BUBBLE_ROUNDING = 0.15     # Corner rounding radius
BORDER_WIDTH = 1.2         # Default border thickness

# Font sizes
FONT_TITLE = 18
FONT_SUBTITLE = 10
FONT_LAYER_LABEL = 10
FONT_BUBBLE_MAIN = 8
FONT_BUBBLE_DETAIL = 7.0
FONT_ARROW_LABEL = 6.5
FONT_PRINCIPLES = 6.5

# Colors
COLORS = {
    'core': '#E8F4FD',        # Light blue - core services
    'viz': '#D4EDDA',         # Light green - visualization
    'robot': '#F8D7DA',       # Light red - robot control
    'ui': '#FFF3CD',          # Light yellow - UI
    'comm': '#E2E3E5',        # Gray - communication/event flow
    'external': '#F5F5F5',    # Light gray - external/assets
}

# =============================================================================
# HELPER FUNCTIONS
# =============================================================================

def draw_bubble(ax, x, y, w, h, text, color, fontsize=8, bold=False,
                border_color='black', border_width=1.2, pad=0.02, rounding=0.15):
    """
    Draw a rounded rectangle bubble with text inside.

    Parameters:
        ax: matplotlib axes
        x, y: center position
        w, h: width and height
        text: text to display (supports newlines with \n)
        color: fill color
        fontsize: text size
        bold: whether to bold the text
        border_color: border color
        border_width: border thickness
        pad: internal padding (text to bubble edge)
        rounding: corner rounding radius
    """
    box = FancyBboxPatch((x - w/2, y - h/2), w, h,
                         boxstyle=f"round,pad={pad},rounding_size={rounding}",
                         facecolor=color, edgecolor=border_color, linewidth=border_width)
    ax.add_patch(box)
    weight = 'bold' if bold else 'normal'
    ax.text(x, y, text, fontsize=fontsize, ha='center', va='center',
            fontweight=weight, wrap=True, linespacing=0.9)
    return box


def draw_arrow(ax, x1, y1, x2, y2, color='black', style='->', lw=1,
               connectionstyle="arc3,rad=0", label=None, label_offset=(0, 0.15)):
    """
    Draw an arrow between two points with optional label.

    Parameters:
        ax: matplotlib axes
        x1, y1: start position
        x2, y2: end position
        color: arrow color
        style: arrow style
        lw: line width
        connectionstyle: curve style (e.g., "arc3,rad=0.2" for curved)
        label: optional text label
        label_offset: (x, y) offset for label from midpoint
    """
    arrow = FancyArrowPatch((x1, y1), (x2, y2),
                            arrowstyle=style, color=color, lw=lw,
                            connectionstyle=connectionstyle,
                            mutation_scale=10)
    ax.add_patch(arrow)
    if label:
        ax.text((x1+x2)/2 + label_offset[0], (y1+y2)/2 + label_offset[1],
                label, fontsize=6.5, ha='center', va='center',
                color='darkblue', style='italic',
                bbox=dict(boxstyle='round,pad=0.1', facecolor='white',
                         edgecolor='none', alpha=0.8))
    return arrow


# =============================================================================
# MAIN DIAGRAM
# =============================================================================

def create_diagram():
    fig, ax = plt.subplots(1, 1, figsize=(FIGURE_WIDTH, FIGURE_HEIGHT))
    ax.set_xlim(0, 12)
    ax.set_ylim(0, 22)
    ax.axis('off')

    # ==================== TITLE ====================
    ax.text(6, LAYER_Y['title'], 'Hatch (孵) — Top-Level Platform Architecture',
            fontsize=FONT_TITLE, fontweight='bold', ha='center', va='center')
    ax.text(6, LAYER_Y['title'] - 0.6,
            'A Derived Architecture: Single Process, Single Memory Space, Event-Driven',
            fontsize=FONT_SUBTITLE, ha='center', va='center', style='italic', color='gray')

    # ==================== LAYER 1: APPLICATION ====================
    ax.text(0.3, LAYER_Y['app'] + 0.3, 'APPLICATION',
            fontsize=FONT_LAYER_LABEL, fontweight='bold', color='black')

    draw_bubble(ax, 6.0, LAYER_Y['app'] - 0.3, 7, 0.8,
                'MainWindow\nCreates & owns all services, engine, robots, UI\nOrchestrates, does NOT hold business logic',
                COLORS['ui'], fontsize=FONT_BUBBLE_MAIN, bold=True,
                border_color='black', border_width=2, pad=BUBBLE_PAD)

    # ==================== LAYER 2: CORE SERVICES ====================
    ax.text(0.3, LAYER_Y['core'] + 0.3, 'CORE SERVICES',
            fontsize=FONT_LAYER_LABEL, fontweight='bold', color='#0C5460')

    # StateChannel
    draw_bubble(ax, 1.5, LAYER_Y['core'] - 0.4, 2.75, 1.2,
                'StateChannel\nPub/Sub events\nHistory & timestamps\nDecoupled\ncommunication',
                COLORS['core'], fontsize=FONT_BUBBLE_DETAIL, bold=True,
                border_color='#0C5460', pad=BUBBLE_PAD)

    # TransformRegistry
    draw_bubble(ax, 4.5, LAYER_Y['core'] - 0.4, 2.75, 1.2,
                'TransformRegistry\nAll spatial poses\nin one place\nLazy evaluation\nCache + callbacks',
                COLORS['core'], fontsize=FONT_BUBBLE_DETAIL, bold=True,
                border_color='#0C5460', pad=BUBBLE_PAD)

    # KinematicModel
    draw_bubble(ax, 7.5, LAYER_Y['core'] - 0.4, 2.75, 1.2,
                'KinematicModel\nURDF parsing\nForward kinematics\nTrue root detection',
                COLORS['core'], fontsize=FONT_BUBBLE_DETAIL, bold=True,
                border_color='#0C5460', pad=BUBBLE_PAD)

    # MeshLoader
    draw_bubble(ax, 10.5, LAYER_Y['core'] - 0.4, 2.75, 1.2,
                'MeshLoader\nLoad STL/OBJ meshes\nColor extraction',
                COLORS['core'], fontsize=FONT_BUBBLE_DETAIL, bold=True,
                border_color='#0C5460', pad=BUBBLE_PAD)

    # Arrows from MainWindow to Core
    draw_arrow(ax, 10, LAYER_Y['app'] - 0.4, 3.5, LAYER_Y['core'] + 0.35,
               color='gray', lw=1, label='creates')
    draw_arrow(ax, 10, LAYER_Y['app'] - 0.4, 8.5, LAYER_Y['core'] + 0.35, color='gray', lw=1)
    draw_arrow(ax, 10, LAYER_Y['app'] - 0.4, 13.5, LAYER_Y['core'] + 0.35, color='gray', lw=1)
    draw_arrow(ax, 10, LAYER_Y['app'] - 0.4, 18, LAYER_Y['core'] + 0.3, color='gray', lw=1)

    # ==================== LAYER 3: ROBOT SYSTEM ====================
    ax.text(0.3, LAYER_Y['robot'] + 0.8, 'ROBOT SYSTEM',
            fontsize=FONT_LAYER_LABEL, fontweight='bold', color='#721C24')

    # RobotManager
    draw_bubble(ax, 1.5, LAYER_Y['robot'] + 0.1, 2.0, 1.2,
                'RobotManager\nRobot lifecycle\nURDF loading\nMode switching',
                COLORS['robot'], fontsize=FONT_BUBBLE_DETAIL, bold=True,
                border_color='#721C24', pad=BUBBLE_PAD)

    # SimulatedRobot
    draw_bubble(ax, 4.5, LAYER_Y['robot'] + 0.1, 2.7, 1.2,
                'SimulatedRobot\nLocal IK solver\nPure Python simulation\nNo Qt',
                COLORS['robot'], fontsize=FONT_BUBBLE_DETAIL, bold=True,
                border_color='#721C24',pad=BUBBLE_PAD)

    # RealRobot
    draw_bubble(ax, 7.5, LAYER_Y['robot'] + 0.1, 2.5, 1.2,
                'RealRobot\nRTDE hardware bridge\nQt for signal handling',
                COLORS['robot'], fontsize=FONT_BUBBLE_DETAIL, bold=True,
                border_color='#721C24',pad=BUBBLE_PAD)

    # CommandHandler
    draw_bubble(ax, 10.5, LAYER_Y['robot'] + 0.1, 2.5, 1.2,
                'CommandHandler\nRoutes commands\nto active robot\nJOINT_COMMAND,\nCARTESIAN_COMMAND',
                COLORS['robot'], fontsize=FONT_BUBBLE_DETAIL, bold=True,
                border_color='#721C24', pad=BUBBLE_PAD)

    # StateHandler
    draw_bubble(ax, 6, LAYER_Y['robot'] - 1.5, 9.5, 1.0,
                'StateHandler\nSingle owner of model + registry updates\nReceives ROBOT_STATE → updates KinematicModel → updates TransformRegistry',
                COLORS['robot'], fontsize=FONT_BUBBLE_DETAIL, bold=True,
                border_color='#721C24', pad=BUBBLE_PAD)

    # Arrows from MainWindow to Robot
    draw_arrow(ax, 10, LAYER_Y['app'] - 0.4, 4, LAYER_Y['robot'] + 0.35,
               color='gray', lw=1, label='creates')
    draw_arrow(ax, 10, LAYER_Y['app'] - 0.4, 8.5, LAYER_Y['robot'] + 0.35, color='gray', lw=1)
    draw_arrow(ax, 10, LAYER_Y['app'] - 0.4, 12.5, LAYER_Y['robot'] + 0.35, color='gray', lw=1)
    draw_arrow(ax, 10, LAYER_Y['app'] - 0.4, 17, LAYER_Y['robot'] + 0.35, color='gray', lw=1)

    # Arrows from Robot components to StateHandler
    draw_arrow(ax, 4, LAYER_Y['robot'] - 0.35, 10, LAYER_Y['robot'] - 0.6,
               color='#721C24', lw=1, label='loads URDF')
    draw_arrow(ax, 8.5, LAYER_Y['robot'] - 0.35, 10, LAYER_Y['robot'] - 0.6, color='#721C24', lw=1)
    draw_arrow(ax, 12.5, LAYER_Y['robot'] - 0.35, 10, LAYER_Y['robot'] - 0.6, color='#721C24', lw=1)

    # ==================== LAYER 4: VISUALIZATION ====================
    ax.text(0.3, LAYER_Y['viz'] + 0.3, 'VISUALIZATION',
            fontsize=FONT_LAYER_LABEL, fontweight='bold', color='#155724')

    # VisualizerEngine
    draw_bubble(ax, 5, LAYER_Y['viz'], 4.5, 0.7,
                'VisualizerEngine\nVTK render window\n60Hz timer\nLazy rendering (sleeps when idle)',
                COLORS['viz'], fontsize=FONT_BUBBLE_DETAIL, bold=True,
                border_color='#155724', pad=BUBBLE_PAD)

    # KinematicDisplay
    draw_bubble(ax, 11, LAYER_Y['viz'], 4.5, 0.7,
                'KinematicDisplay\nVTK actors for each link\nTransformRegistry callbacks\n_needs_render flag',
                COLORS['viz'], fontsize=FONT_BUBBLE_DETAIL, bold=True,
                border_color='#155724', pad=BUBBLE_PAD)

    # CameraManager
    draw_bubble(ax, 17, LAYER_Y['viz'], 4.5, 0.7,
                'CameraManager\nDiscovers cameras from URDF\nOne CameraPipeline per camera\n30 FPS each',
                COLORS['viz'], fontsize=FONT_BUBBLE_DETAIL, bold=True,
                border_color='#155724', pad=BUBBLE_PAD)

    # Arrows from MainWindow to Viz
    draw_arrow(ax, 10, LAYER_Y['app'] - 0.4, 5, LAYER_Y['viz'] + 0.35,
               color='gray', lw=1, label='creates')
    draw_arrow(ax, 10, LAYER_Y['app'] - 0.4, 11, LAYER_Y['viz'] + 0.35, color='gray', lw=1)
    draw_arrow(ax, 10, LAYER_Y['app'] - 0.4, 17, LAYER_Y['viz'] + 0.35, color='gray', lw=1)

    # Arrow from StateHandler to KinematicDisplay
    draw_arrow(ax, 10, LAYER_Y['robot'] - 1.2, 11, LAYER_Y['viz'] + 0.35,
               color='#155724', lw=1.5, label='transforms')

    # ==================== LAYER 5: USER INTERFACE ====================
    ax.text(0.3, LAYER_Y['ui'] + 0.3, 'USER INTERFACE',
            fontsize=FONT_LAYER_LABEL, fontweight='bold', color='#856404')

    # UIBuilder
    draw_bubble(ax, 3.5, LAYER_Y['ui'], 3, 0.6,
                'UIBuilder\nMenus, toolbars, docks\nPanel construction',
                COLORS['ui'], fontsize=FONT_BUBBLE_DETAIL, pad=BUBBLE_PAD)

    # MotionContainer
    draw_bubble(ax, 7.5, LAYER_Y['ui'], 3.5, 0.6,
                'MotionContainer\nJointControlPanel\nCartesianControlPanel\nRobotConnectionPanel',
                COLORS['ui'], fontsize=FONT_BUBBLE_DETAIL, bold=True,
                border_color='#856404', pad=BUBBLE_PAD)

    # CameraControlPanel
    draw_bubble(ax, 12, LAYER_Y['ui'], 3.5, 0.6,
                'CameraControlPanel\nSwitch between cameras\nStart/Stop/ROI/Visibility',
                COLORS['ui'], fontsize=FONT_BUBBLE_DETAIL, pad=BUBBLE_PAD)

    # JointFramePanel
    draw_bubble(ax, 16.5, LAYER_Y['ui'], 3, 0.6,
                'JointFramePanel\nShow/hide link frames\nScale & thickness',
                COLORS['ui'], fontsize=FONT_BUBBLE_DETAIL, pad=BUBBLE_PAD)

    # Arrows from MainWindow to UI
    draw_arrow(ax, 10, LAYER_Y['app'] - 0.4, 3.5, LAYER_Y['ui'] + 0.3,
               color='gray', lw=1, label='creates')
    draw_arrow(ax, 10, LAYER_Y['app'] - 0.4, 7.5, LAYER_Y['ui'] + 0.3, color='gray', lw=1)
    draw_arrow(ax, 10, LAYER_Y['app'] - 0.4, 12, LAYER_Y['ui'] + 0.3, color='gray', lw=1)
    draw_arrow(ax, 10, LAYER_Y['app'] - 0.4, 16.5, LAYER_Y['ui'] + 0.3, color='gray', lw=1)

    # ==================== LAYER 6: EVENT FLOW ====================
    ax.text(0.3, LAYER_Y['event_flow'] + 0.3, 'EVENT FLOW',
            fontsize=FONT_LAYER_LABEL, fontweight='bold', color='black')

    draw_bubble(ax, 10, LAYER_Y['event_flow'], 10, 0.9,
                'User moves slider → UI Panel publishes COMMAND → StateChannel distributes → CommandHandler routes → Robot executes → publishes ROBOT_STATE\n→ StateHandler updates KinematicModel → TransformRegistry callbacks → KinematicDisplay sets _needs_render → VisualizerEngine renders\n→ UI Panels receive ROBOT_STATE → update slider positions',
                COLORS['comm'], fontsize=FONT_BUBBLE_DETAIL, bold=True,
                border_color='black', pad=BUBBLE_PAD)

    # ==================== LAYER 7: DATA & ASSETS ====================
    ax.text(0.3, LAYER_Y['data'] + 0.3, 'DATA & ASSETS',
            fontsize=FONT_LAYER_LABEL, fontweight='bold', color='gray')

    # URDF / Xacro
    draw_bubble(ax, 4, LAYER_Y['data'], 3.5, 0.6,
                'URDF / Xacro\nScene description\nLinks, joints, meshes\nPackage:// paths',
                COLORS['external'], fontsize=FONT_BUBBLE_DETAIL, pad=BUBBLE_PAD)

    # URDFPreprocessor
    draw_bubble(ax, 8.5, LAYER_Y['data'], 3.5, 0.6,
                'URDFPreprocessor\nResolves includes\nSubstitutes variables\nExpands macros',
                COLORS['external'], fontsize=FONT_BUBBLE_DETAIL, pad=BUBBLE_PAD)

    # Assets
    draw_bubble(ax, 13, LAYER_Y['data'], 3, 0.6,
                'Assets\nScenes / Robots / Sensors\nUGV / Tools / Meshes',
                COLORS['external'], fontsize=FONT_BUBBLE_DETAIL, pad=BUBBLE_PAD)

    # Hardware
    draw_bubble(ax, 17, LAYER_Y['data'], 3, 0.6,
                'Hardware\nUR Robot (RTDE)\nCameras (USB/Network)',
                COLORS['external'], fontsize=FONT_BUBBLE_DETAIL, pad=BUBBLE_PAD)

    # Arrows between data components
    draw_arrow(ax, 4, LAYER_Y['data'] + 0.3, 8.5, LAYER_Y['data'] + 0.3,
               color='gray', lw=1, label='feeds')
    draw_arrow(ax, 8.5, LAYER_Y['data'] + 0.3, 13, LAYER_Y['data'] + 0.3,
               color='gray', lw=1, label='loads')
    draw_arrow(ax, 13, LAYER_Y['data'] + 0.3, 17, LAYER_Y['data'] + 0.3,
               color='gray', lw=1, label='drives')

    # ==================== PRINCIPLES BANNER ====================
    ax.text(10, LAYER_Y['principles'],
            'Principles: #0 Individuals Before Groups  |  #1 Single Process  |  #2 Event-Driven  |  #3 Visualizer as Mind-Prying Tool  |  #4 Everything in URDF  |  #5 Space = TransformRegistry  |  #6 Time = StateChannel  |  #7 Movements as Models  |  #8 Pure Python  |  #9 UI Separate from Services  |  #10 One Robot Per Session',
            fontsize=FONT_PRINCIPLES, ha='center', va='center', style='italic', color='gray',
            bbox=dict(boxstyle='round,pad=0.3', facecolor='white', edgecolor='gray', alpha=0.8))

    # Save and show
    plt.tight_layout()
    plt.savefig('hatch_top_level_architecture.png', dpi=DPI, bbox_inches='tight',
                facecolor='white', edgecolor='none')
    plt.show()
    print("Diagram saved to hatch_top_level_architecture.png")


if __name__ == "__main__":
    create_diagram()
