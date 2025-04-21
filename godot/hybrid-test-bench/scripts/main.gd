extends Node3D

@onready var hybrid_test_bench: HybridTestBench = $HybridTestBench

func _ready() -> void:
	hybrid_test_bench.move_actuator()
