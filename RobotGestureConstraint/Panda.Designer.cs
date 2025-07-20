﻿namespace iM
{
    partial class cpPanda
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.cbConnectDisconnect = new System.Windows.Forms.CheckBox();
            this.cbReadOnce = new System.Windows.Forms.Button();
            this.btnHome = new System.Windows.Forms.Button();
            this.btnStartControl = new System.Windows.Forms.Button();
            this.btnStop = new System.Windows.Forms.Button();
            this.label2 = new System.Windows.Forms.Label();
            this.ListControlTypes = new System.Windows.Forms.ListBox();
            this.label1 = new System.Windows.Forms.Label();
            this.labelRobotState = new System.Windows.Forms.Label();
            this.SuspendLayout();
            // 
            // cbConnectDisconnect
            // 
            this.cbConnectDisconnect.Appearance = System.Windows.Forms.Appearance.Button;
            this.cbConnectDisconnect.Font = new System.Drawing.Font("Microsoft Sans Serif", 15.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.cbConnectDisconnect.Location = new System.Drawing.Point(39, 36);
            this.cbConnectDisconnect.Name = "cbConnectDisconnect";
            this.cbConnectDisconnect.Size = new System.Drawing.Size(156, 35);
            this.cbConnectDisconnect.TabIndex = 6;
            this.cbConnectDisconnect.Text = "Connect";
            this.cbConnectDisconnect.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            this.cbConnectDisconnect.UseVisualStyleBackColor = true;
            this.cbConnectDisconnect.CheckedChanged += new System.EventHandler(this.cbConnectDisconnect_CheckedChanged);
            // 
            // cbReadOnce
            // 
            this.cbReadOnce.Location = new System.Drawing.Point(39, 77);
            this.cbReadOnce.Name = "cbReadOnce";
            this.cbReadOnce.Size = new System.Drawing.Size(75, 23);
            this.cbReadOnce.TabIndex = 7;
            this.cbReadOnce.Text = "Read once";
            this.cbReadOnce.UseVisualStyleBackColor = true;
            this.cbReadOnce.Click += new System.EventHandler(this.cbReadOnce_CheckedChanged);
            // 
            // btnHome
            // 
            this.btnHome.Location = new System.Drawing.Point(120, 77);
            this.btnHome.Name = "btnHome";
            this.btnHome.Size = new System.Drawing.Size(75, 23);
            this.btnHome.TabIndex = 13;
            this.btnHome.Text = "Home";
            this.btnHome.UseVisualStyleBackColor = true;
            this.btnHome.Click += new System.EventHandler(this.btnHome_Click);
            // 
            // btnStartControl
            // 
            this.btnStartControl.Location = new System.Drawing.Point(201, 77);
            this.btnStartControl.Name = "btnStartControl";
            this.btnStartControl.Size = new System.Drawing.Size(75, 23);
            this.btnStartControl.TabIndex = 15;
            this.btnStartControl.Text = "Start Control";
            this.btnStartControl.UseVisualStyleBackColor = true;
            this.btnStartControl.Click += new System.EventHandler(this.btnStartControl_Click);
            // 
            // btnStop
            // 
            this.btnStop.Font = new System.Drawing.Font("Microsoft Sans Serif", 15F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btnStop.Location = new System.Drawing.Point(201, 36);
            this.btnStop.Name = "btnStop";
            this.btnStop.Size = new System.Drawing.Size(75, 35);
            this.btnStop.TabIndex = 26;
            this.btnStop.Text = "Stop";
            this.btnStop.UseVisualStyleBackColor = true;
            this.btnStop.Click += new System.EventHandler(this.btnStop_Click);
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label2.Location = new System.Drawing.Point(35, 9);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(26, 20);
            this.label2.TabIndex = 10;
            this.label2.Text = "IP";
            // 
            // ListControlTypes
            // 
            this.ListControlTypes.FormattingEnabled = true;
            this.ListControlTypes.Items.AddRange(new object[] {
            "gesture mode",
            "Gravity compensation",
            "Cartesian Impedance",
            "Cartesian Teleimpedance",
            "Cartesian force"});
            this.ListControlTypes.Location = new System.Drawing.Point(39, 126);
            this.ListControlTypes.Name = "ListControlTypes";
            this.ListControlTypes.Size = new System.Drawing.Size(237, 56);
            this.ListControlTypes.TabIndex = 27;
            this.ListControlTypes.SelectedIndexChanged += new System.EventHandler(this.ListControlTypes_SelectedIndexChanged);
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label1.Location = new System.Drawing.Point(35, 103);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(106, 20);
            this.label1.TabIndex = 28;
            this.label1.Text = "Control type";
            // 
            // labelRobotState
            // 
            this.labelRobotState.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.labelRobotState.Location = new System.Drawing.Point(201, 9);
            this.labelRobotState.Name = "labelRobotState";
            this.labelRobotState.Size = new System.Drawing.Size(117, 20);
            this.labelRobotState.TabIndex = 30;
            // 
            // cpPanda
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(1000, 700);
            this.Controls.Add(this.labelRobotState);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.ListControlTypes);
            this.Controls.Add(this.btnStop);
            this.Controls.Add(this.btnStartControl);
            this.Controls.Add(this.btnHome);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.cbReadOnce);
            this.Controls.Add(this.cbConnectDisconnect);
            this.Name = "cpPanda";
            this.Text = "Franka";
            this.Load += new System.EventHandler(this.cpFranka_Load);
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.CheckBox cbConnectDisconnect;
        private System.Windows.Forms.Button cbReadOnce;
        private System.Windows.Forms.Button btnHome;
        private System.Windows.Forms.Button btnStartControl;
        private System.Windows.Forms.Button btnStop;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.ListBox ListControlTypes;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Label labelRobotState;
    }
}