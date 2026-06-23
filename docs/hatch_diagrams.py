
"""
Hatch Platform Architecture Diagrams Generator
===============================================

Generates two diagrams:
1. A3 Landscape Architecture Diagram (no Event Flow, no principle numbers)
2. A4 Portrait Event Flow Diagram

Usage:
    python hatch_diagrams.py

Output:
    hatch_architecture_a3.png / .pdf
    hatch_event_flow.png / .pdf

Modify the CONFIGURATION section to adjust sizes, colors, and spacing.
"""

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
import numpy as np

# =============================================================================
# GLOBAL FONT SETTINGS
# =============================================================================

plt.rcParams['font.sans-serif'] = ['Noto Sans CJK SC', 'WenQuanYi Micro Hei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False
plt.rcParams['figure.autolayout'] = False

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
            fontweight=weight, wrap=True, linespacing=1.4)
    return box


def draw_arrow(ax, x1, y1, x2, y2, color='black', style='->', lw=1,
               connectionstyle="arc3,rad=0", label=None, 
               label_offset=(0, 0.15), label_fraction=0.5):
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
        # Calculate position along the arrow
        label_x = x1 + (x2 - x1) * label_fraction
        label_y = y1 + (y2 - y1) * label_fraction
        
        ax.text(label_x + label_offset[0], label_y + label_offset[1],
                label, fontsize=7, ha='center', va='center',
                color='darkblue', style='italic',
                bbox=dict(boxstyle='round,pad=0.1', facecolor='white',
                         edgecolor='none', alpha=0.8))
    return arrow


# =============================================================================
# DIAGRAM 1: A3 LANDSCAPE ARCHITECTURE
# =============================================================================

def create_architecture_diagram():
    """Create the A3 Landscape Architecture Diagram."""

    # CONFIGURATION - Adjust these values
    FIGURE_WIDTH = 16.54         # A3 width in inches (landscape)
    FIGURE_HEIGHT = 11.69        # A3 height in inches
    DPI = 300                    # Output resolution
    CENTER_X = FIGURE_WIDTH / 2  # 8.27

    # Layer Y-positions (from bottom to top)
    LAYER_Y = {
        'principles': 0.3,
        'data': 1.6,
        'ui': 3.1,
        'viz': 4.6,
        'robot': 6.9,
        'core': 8.9,
        'app': 10.3,      # MainWindow layer
        'title': 11.3,    # Title (moved up to avoid overlap)
    }

    # Bubble sizing
    BUBBLE_HEIGHT = 0.9
    BUBBLE_WIDTH = 3.2
    BUBBLE_PAD = 0.005
    BUBBLE_ROUNDING = 0.15
    BORDER_WIDTH = 1.2

    # Font sizes
    FONT_TITLE = 20
    FONT_SUBTITLE = 11
    FONT_LAYER_LABEL = 11
    FONT_BUBBLE_MAIN = 11
    FONT_BUBBLE_DETAIL = 11
    FONT_ARROW_LABEL = 7
    FONT_PRINCIPLES = 10

    # Colors
    COLORS = {
        'core': '#E8F4FD',        # Light blue - core services
        'viz': '#D4EDDA',         # Light green - visualization
        'robot': '#F8D7DA',       # Light red - robot control
        'ui': '#FFF3CD',          # Light yellow - UI
        'external': '#F5F5F5',    # Light gray - external/assets
    }

    # Create figure
    fig, ax = plt.subplots(1, 1, figsize=(FIGURE_WIDTH, FIGURE_HEIGHT),
                           dpi=DPI, constrained_layout=False)

    # Remove ALL padding - axes fill entire figure
    fig.subplots_adjust(left=0, right=1, bottom=0, top=1)
    ax.margins(0)
    ax.set_xlim(0, FIGURE_WIDTH)
    ax.set_ylim(0, FIGURE_HEIGHT)
    ax.axis('off')

    # ==================== TITLE ====================
    ax.text(CENTER_X, LAYER_Y['title'], 'Hatch (孵) — Top-Level Platform Architecture',
            fontsize=FONT_TITLE, fontweight='bold', ha='center', va='center')
    ax.text(CENTER_X, LAYER_Y['title'] - 0.35,
            'A Derived Architecture: Single Process, Single Memory Space, Event-Driven',
            fontsize=FONT_SUBTITLE, ha='center', va='center', style='italic', color='gray')

    # ==================== LAYER 1: APPLICATION ====================
    ax.text(0.5, LAYER_Y['app'] + 0.35, 'APPLICATION',
            fontsize=FONT_LAYER_LABEL, fontweight='bold', color='black')

    mainwindow_x = CENTER_X
    mainwindow_y = LAYER_Y['app'] - 0.3
    mainwindow_w = 6
    mainwindow_h = 1.0

    draw_bubble(ax, mainwindow_x, mainwindow_y, mainwindow_w, mainwindow_h,
                'MainWindow\nCreates & owns all services, engine, robots, UI\nOrchestrates, does NOT hold business logic',
                COLORS['ui'], fontsize=FONT_BUBBLE_MAIN, bold=True,
                border_color='black', border_width=2, pad=BUBBLE_PAD)

    mainwindow_bottom = mainwindow_y - mainwindow_h/2

    # ==================== LAYER 2: CORE SERVICES ====================
    ax.text(0.5, LAYER_Y['core'] + 0.35, 'CORE SERVICES',
            fontsize=FONT_LAYER_LABEL, fontweight='bold', color='#0C5460')

    core_bubbles = [
        (3.3, 'StateChannel\nPub/Sub events\nHistory & timestamps\nDecoupled communication'),
        (6.6, 'TransformRegistry\nAll spatial poses\nin one place\nLazy evaluation\nCache + callbacks'),
        (9.9, 'KinematicModel\nURDF parsing\nForward kinematics\nTrue root detection'),
        (13.4, 'MeshLoader\nLoad STL/OBJ meshes\nColor extraction'),
    ]

    for i, (x, text) in enumerate(core_bubbles):
        y = LAYER_Y['core'] - 0.25
        w = 2.8
        h = 1.1
        draw_bubble(ax, x, y, w, h, text, COLORS['core'], 
                   fontsize=FONT_BUBBLE_DETAIL, bold=True,
                   border_color='#0C5460', pad=BUBBLE_PAD)
        draw_arrow(ax, mainwindow_x, mainwindow_bottom, x, y + h/2,
                   color='gray', lw=1, label='creates' if i == 0 else None, label_fraction=0.7)

    # ==================== LAYER 3: ROBOT SYSTEM ====================
    ax.text(0.5, LAYER_Y['robot'] + 1, 'ROBOT SYSTEM',
            fontsize=FONT_LAYER_LABEL, fontweight='bold', color='#721C24')

    robot_bubbles = [
        (3.3, 'RobotManager\nRobot lifecycle\nURDF loading\nMode switching'),
        (6.6, 'SimulatedRobot\nLocal IK solver\nPure Python simulation\nNo Qt'),
        (9.9, 'RealRobot\nRTDE hardware bridge\nQt for signal handling'),
        (13.4, 'CommandHandler\nRoutes commands\nto active robot\nJOINT_COMMAND,\nCARTESIAN_COMMAND'),
    ]

    robot_positions = []
    for i, (x, text) in enumerate(robot_bubbles):
        y = LAYER_Y['robot'] + 0.35
        w = 2.8
        h = 1.1
        draw_bubble(ax, x, y, w, h, text, COLORS['robot'],
                   fontsize=FONT_BUBBLE_DETAIL, bold=True,
                   border_color='#721C24', pad=BUBBLE_PAD)
        robot_positions.append((x, y, w, h))
        draw_arrow(ax, mainwindow_x, mainwindow_bottom, x, y + h/2,
                   color='gray', lw=1, label='creates' if i == 0 else None, label_fraction=0.67, label_offset=(0, -0.1))

    # StateHandler
    statehandler_x = CENTER_X
    statehandler_y = LAYER_Y['robot'] - 1.1
    statehandler_w = 8
    statehandler_h = 1.0

    draw_bubble(ax, statehandler_x, statehandler_y, statehandler_w, statehandler_h,
                'StateHandler\nSingle owner of model + registry updates\nReceives ROBOT_STATE → updates KinematicModel → updates TransformRegistry',
                COLORS['robot'], fontsize=FONT_BUBBLE_DETAIL, bold=True,
                border_color='#721C24', pad=BUBBLE_PAD)

    # Only RobotManager loads URDF and feeds StateHandler
    robotmanager_x, robotmanager_y, robotmanager_w, robotmanager_h = robot_positions[0]

    draw_arrow(ax, robotmanager_x, robotmanager_y - robotmanager_h/2, 
            statehandler_x, statehandler_y + statehandler_h/2,
            color='#721C24', lw=1.5, 
            label='loads URDF',
            label_fraction=0.65, 
            label_offset=(0, 0.1))

    # ==================== LAYER 4: VISUALIZATION ====================
    ax.text(0.5, LAYER_Y['viz'] + 0.4, 'VISUALIZATION',
            fontsize=FONT_LAYER_LABEL, fontweight='bold', color='#155724')

    viz_bubbles = [
        (3.8, 'VisualizerEngine\nVTK render window\n60Hz timer\nLazy rendering (sleeps when idle)'),
        (8.3, 'KinematicDisplay\nVTK actors for each link\nTransformRegistry callbacks\n_needs_render flag'),
        (12.8, 'CameraManager\nDiscovers cameras from URDF\nOne CameraPipeline per camera\n30 FPS each'),
    ]

    viz_positions = []
    for i, (x, text) in enumerate(viz_bubbles):
        y = LAYER_Y['viz'] - 0.2
        w = 3.8
        h = 1.1
        draw_bubble(ax, x, y, w, h, text, COLORS['viz'],
                   fontsize=FONT_BUBBLE_DETAIL, bold=True,
                   border_color='#155724', pad=BUBBLE_PAD)
        viz_positions.append((x, y, w, h))
        draw_arrow(ax, mainwindow_x, mainwindow_bottom, x, y + h/2,
                   color='gray', lw=1, label='creates' if i == 0 else None, label_fraction=0.8)

    # Arrow from StateHandler to KinematicDisplay
    draw_arrow(ax, statehandler_x, statehandler_y - statehandler_h/2,
               8.0, viz_positions[1][1] + viz_positions[1][3]/2,
               color='#155724', lw=1.5, label='transforms', label_fraction=0.5, label_offset=(-0.25, 0.01))

    # ==================== LAYER 5: USER INTERFACE ====================
    ax.text(0.5, LAYER_Y['ui'] + 0.4, 'USER INTERFACE',
            fontsize=FONT_LAYER_LABEL, fontweight='bold', color='#856404')

    ui_bubbles = [
        (3.3, 'UIBuilder\nMenus, toolbars, docks\nPanel construction'),
        (6.6, 'MotionContainer\nJointControlPanel\nCartesianControlPanel\nRobotConnectionPanel'),
        (9.9, 'CameraControlPanel\nSwitch between cameras\nStart/Stop/ROI/Visibility'),
        (13.4, 'JointFramePanel\nShow/hide link frames\nScale & thickness'),
    ]

    for i, (x, text) in enumerate(ui_bubbles):
        y = LAYER_Y['ui'] - 0.2
        w = 2.8
        h = 1.0
        draw_bubble(ax, x, y, w, h, text, COLORS['ui'],
                   fontsize=FONT_BUBBLE_DETAIL, bold=True,
                   border_color='#856404', pad=BUBBLE_PAD)
        draw_arrow(ax, mainwindow_x, mainwindow_bottom, x, y + h/2,
                   color='gray', lw=1, label='creates' if i == 0 else None, label_fraction=0.97)

    # ==================== LAYER 6: DATA & ASSETS ====================
    ax.text(0.5, LAYER_Y['data'] + 0.4, 'DATA & ASSETS',
            fontsize=FONT_LAYER_LABEL, fontweight='bold', color='gray')

    data_bubbles = [
        (3.3, 'URDF / Xacro\nScene description\nLinks, joints, meshes\nPackage:// paths'),
        (6.6, 'URDFPreprocessor\nResolves includes\nSubstitutes variables\nExpands macros'),
        (9.9, 'Assets\nScenes / Robots / Sensors\nUGV / Tools / Meshes'),
        (13.4, 'Hardware\nUR Robot (RTDE)\nCameras (USB/Network)'),
    ]

    for i, (x, text) in enumerate(data_bubbles):
        y = LAYER_Y['data'] - 0.2
        w = 2.8
        h = 1.0
        draw_bubble(ax, x, y, w, h, text, COLORS['external'],
                   fontsize=FONT_BUBBLE_DETAIL, pad=BUBBLE_PAD)
        if i < len(data_bubbles) - 1:
            next_x = data_bubbles[i+1][0]
            draw_arrow(ax, x + w/2, y, next_x - 2.8/2, y,
                       color='gray', lw=1, 
                       label='feeds' if i == 0 else ('loads' if i == 1 else 'drives'))

    # ==================== PRINCIPLES - NO NUMBERS ====================
    principles_lines = [
        'Individuals before groups.  |  A single process, single memory space.  |  Event-driven, no polling. | The visualizer as a mind-prying tool — see everything, hide nothing.  |  Everything in URDF.',
        'Space is the TransformRegistry.  |  Time is the StateChannel.  |  Movements are models. Pure Python.  |  The UI separate from services.  |  One robot, one session.'
    ]

    for i, line in enumerate(principles_lines):
        ax.text(CENTER_X, LAYER_Y['principles'] + i * 0.25, line,
                fontsize=FONT_PRINCIPLES, ha='center', va='center', 
                style='italic', color='gray')

    # Save
    plt.savefig('hatch_architecture_a3.png', 
                dpi=DPI, bbox_inches=None, pad_inches=0,
                facecolor='white', edgecolor='none')
    plt.savefig('hatch_architecture_a3.pdf', 
                format='pdf', bbox_inches=None, pad_inches=0,
                facecolor='white', edgecolor='none')
    print("Architecture Diagram saved!")
    print(f"  PNG: hatch_architecture_a3.png")
    print(f"  PDF: hatch_architecture_a3.pdf")
    print(f"  Size: {int(FIGURE_WIDTH*DPI)} x {int(FIGURE_HEIGHT*DPI)} pixels")
    plt.close()


# =============================================================================
# DIAGRAM 2: A4 PORTRAIT EVENT FLOW
# =============================================================================

def create_event_flow_diagram():
    """Create the A4 Portrait Event Flow Diagram."""

    # CONFIGURATION
    FIGURE_WIDTH = 8.27
    FIGURE_HEIGHT = 11.69
    DPI = 300
    CENTER_X = FIGURE_WIDTH / 2

    BUBBLE_PAD = 0.005
    BUBBLE_ROUNDING = 0.15
    BORDER_WIDTH = 1.2

    FONT_TITLE = 16
    FONT_SUBTITLE = 10
    FONT_STEP = 9
    FONT_ARROW_LABEL = 7.5

    COLORS = {
        'user': '#FFF3CD',      # Yellow - user action
        'ui': '#E8F4FD',        # Blue - UI
        'channel': '#D4EDDA',   # Green - StateChannel
        'handler': '#F8D7DA',   # Red - handlers
        'robot': '#E2E3E5',     # Gray - robot
        'state': '#D1ECF1',     # Cyan - state update
        'render': '#F5F5F5',    # Light gray - render
    }

    # Create figure
    fig, ax = plt.subplots(1, 1, figsize=(FIGURE_WIDTH, FIGURE_HEIGHT),
                           dpi=DPI, constrained_layout=False)
    fig.subplots_adjust(left=0, right=1, bottom=0, top=1)
    ax.margins(0)
    ax.set_xlim(0, FIGURE_WIDTH)
    ax.set_ylim(0, FIGURE_HEIGHT)
    ax.axis('off')

    # TITLE
    ax.text(CENTER_X, 11.0, 'Hatch Event Flow',
            fontsize=FONT_TITLE, fontweight='bold', ha='center', va='center')
    ax.text(CENTER_X, 10.6,
            'The Complete Cycle: User Input → Command → Execution → State → Render → Display',
            fontsize=FONT_SUBTITLE, ha='center', va='center', style='italic', color='gray')

    # Steps from top to bottom
    steps = [
        (9.8, '1. USER', 'User moves slider\n(sets target joint angle or pose)', COLORS['user'], 'black'),
        (8.8, '2. UI PANEL', 'UI Panel receives input\npublishes COMMAND event', COLORS['ui'], '#0C5460'),
        (7.8, '3. STATECHANNEL', 'StateChannel distributes\nCOMMAND to all subscribers', COLORS['channel'], '#155724'),
        (6.8, '4. COMMANDHANDLER', 'CommandHandler routes COMMAND\nto active robot (Simulated or Real)', COLORS['handler'], '#721C24'),
        (5.8, '5. ROBOT', 'Robot executes movement\npublishes ROBOT_STATE event', COLORS['robot'], 'gray'),
        (4.8, '6. STATECHANNEL', 'StateChannel distributes\nROBOT_STATE to all subscribers', COLORS['channel'], '#155724'),
        (3.8, '7. STATEHANDLER', 'StateHandler receives ROBOT_STATE\nupdates KinematicModel', COLORS['state'], '#0C5460'),
        (2.8, '8. TRANSFORMREGISTRY', 'TransformRegistry updates\nfires callbacks to all listeners', COLORS['state'], '#0C5460'),
        (1.8, '9. KINEMATICDISPLAY', 'KinematicDisplay receives new transforms\nsets _needs_render = True', COLORS['render'], '#155724'),
        (0.8, '10. VISUALIZERENGINE', 'VisualizerEngine renders frame\n3D view updates', COLORS['render'], '#155724'),
    ]

    # Draw bubbles
    for y, title, text, color, border in steps:
        draw_bubble(ax, CENTER_X, y, 6, 0.7, 
                   f'{title}\n{text}', color, 
                   fontsize=FONT_STEP, bold=True,
                   border_color=border, pad=BUBBLE_PAD)

    # Draw arrows between steps
    for i in range(len(steps) - 1):
        y1 = steps[i][0] - 0.35
        y2 = steps[i+1][0] + 0.35
        draw_arrow(ax, CENTER_X, y1, CENTER_X, y2, color='black', lw=1.5)

    # Save
    plt.savefig('hatch_event_flow.png', 
                dpi=DPI, bbox_inches=None, pad_inches=0,
                facecolor='white', edgecolor='none')
    plt.savefig('hatch_event_flow.pdf', 
                format='pdf', bbox_inches=None, pad_inches=0,
                facecolor='white', edgecolor='none')
    print("Event Flow Diagram saved!")
    print(f"  PNG: hatch_event_flow.png")
    print(f"  PDF: hatch_event_flow.pdf")
    print(f"  Size: {int(FIGURE_WIDTH*DPI)} x {int(FIGURE_HEIGHT*DPI)} pixels")
    plt.close()


# =============================================================================
# MAIN
# =============================================================================

if __name__ == "__main__":
    print("Generating Hatch Diagrams...")
    print("=" * 50)

    create_architecture_diagram()
    print()
    create_event_flow_diagram()

    print()
    print("=" * 50)
    print("All diagrams generated successfully!")
