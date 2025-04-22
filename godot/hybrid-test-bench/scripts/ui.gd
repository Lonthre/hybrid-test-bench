class_name Ui
extends Control


@onready var vertical_force: Label = %VerticalForce
@onready var horizontal_force: Label = %HorizontalForce
@onready var vertical_displacement: Label = %VerticalDisplacement
@onready var horizontal_displacement: Label = %HorizontalDisplacement


func update_vertical_force(new_value: float) -> void:
	vertical_force.text = str(snappedf(new_value, 0.0001))
	
func update_horizontal_force(new_value: float) -> void:
	horizontal_force.text = str(snappedf(new_value, 0.0001))
	
func update_vertical_displacement(new_value: float) -> void:
	vertical_displacement.text = str(snappedf(new_value, 0.0001))
	
func update_horizontal_displacement(new_value: float) -> void:
	horizontal_displacement.text = str(snappedf(new_value, 0.0001))
