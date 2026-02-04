#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
itute of Technology, Hibikino-Musashi@Home）
#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
）
#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
連）
#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
up/yasmin.git
#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
テートマシンを定義するのに用いるクラスをインポート
#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
werPub#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
テート）
#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
r2_main, search#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
クラスを継承）
#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
クラス
#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
"
#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
ストラクタをオーバーライド(引数は，'ノード名')
#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
スタンを生成
#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
["EXIT","FAILED"], initial_state="MR1")#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
トを追加
#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
名
#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
(node=self),  # 実行するステート
#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
移するステートを定義する辞書
#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
,  # "outcome1"が返ってきたらBARステートに遷移
#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
トを追加
#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
名
#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
(node=self),  # 実行するステート
#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
移するステートを定義する辞書
#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
,  # "outcome2"が返ってきたらFOOステートに遷移
#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
ート名
#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
e(node=self),  # 実行するステート
#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
移するステートを定義する辞書
#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
 "outcome2"が返ってきたらFOOステートに遷移
#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
,
#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
"
#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
シンの情報をパブリッシュ
#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
M_MAIN", fsm=sm)#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
te Machine finished with outcome: " + outcome)#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
ブラリの初期化
#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
スタント生成
#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibiki
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine #ステートマシンを定義するのに
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import mr1, mr2, mr3


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed1": "MR2",  # "outcome1"が返ってきたら
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed2": "MR3",  # "outcome2"が返ってきたら
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="MR3",  # ステート名
            state=mr3.MR3State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞
                "succeed3": "EXIT"  # "outcome2"が返ってきたら
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with ou


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
