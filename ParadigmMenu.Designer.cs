namespace iM
{
    partial class cpParadigmMenu
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
        private void InitializeComponent() 
        {
            this.smParadigmMenu = new iM.ScopeMonitor();
            this.state = new System.Windows.Forms.Label();
            this.motion_activated = new System.Windows.Forms.Label();
            this.output = new System.Windows.Forms.Label();
            this.info = new System.Windows.Forms.Label();
            this.orientation = new System.Windows.Forms.Label();
            this.axis_info = new System.Windows.Forms.Label();
            this.button1 = new System.Windows.Forms.Button();
            this.SuspendLayout();
            // 
            // cpParadigmMenu
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(1050, 310);
            this.Controls.Add(this.state);
            this.Controls.Add(this.axis_info);
            this.Controls.Add(this.motion_activated);
            this.Controls.Add(this.orientation);
            this.Controls.Add(this.output);
            this.Controls.Add(this.info);
            this.Controls.Add(this.button1 );
            this.Name = "cpParadigmMenu";
            this.ResumeLayout(false);
            this.PerformLayout();
            // 
            // state label
            // 
            this.state.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Top)
            | System.Windows.Forms.AnchorStyles.Left)
            | System.Windows.Forms.AnchorStyles.Right)));
            this.state.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.state.Font = new System.Drawing.Font("Microsoft Sans Serif", 15.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.state.Location = new System.Drawing.Point(0, 10);
            this.state.Name = "Menu state";
            this.state.Size = new System.Drawing.Size(600, 50);
            this.state.TabIndex = 0;
            //this.state.BackColor = System.Drawing.SystemColors.Window;
            this.state.TextAlign = System.Drawing.ContentAlignment.BottomCenter;
            // 
            // axis information label
            // 
            this.axis_info.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom)
            | System.Windows.Forms.AnchorStyles.Left)
            | System.Windows.Forms.AnchorStyles.Right)));
            this.axis_info.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.axis_info.Font = new System.Drawing.Font("Microsoft Sans Serif", 15.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.axis_info.Location = new System.Drawing.Point(0, 50);
            this.axis_info.Name = "In motion";
            this.axis_info.Size = new System.Drawing.Size(600, 100);
            this.axis_info.TabIndex = 0;
            this.axis_info.Text = "axis_info";
            this.axis_info.TextAlign = System.Drawing.ContentAlignment.BottomCenter;

            // 
            // motion activated label
            // 
            this.motion_activated.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom)
            | System.Windows.Forms.AnchorStyles.Left)
            | System.Windows.Forms.AnchorStyles.Right)));
            this.motion_activated.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.motion_activated.Font = new System.Drawing.Font("Microsoft Sans Serif", 15.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.motion_activated.Location = new System.Drawing.Point(0, 160);
            this.motion_activated.Name = "In motion";
            this.motion_activated.Size = new System.Drawing.Size(600, 50);
            this.motion_activated.TabIndex = 0;
            this.motion_activated.Text = "motion_activated";
            this.motion_activated.TextAlign = System.Drawing.ContentAlignment.BottomCenter;
            // 
            // orientation label
            // 
            this.orientation.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom)
            | System.Windows.Forms.AnchorStyles.Left)
            | System.Windows.Forms.AnchorStyles.Right)));
            this.orientation.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.orientation.Font = new System.Drawing.Font("Microsoft Sans Serif", 15.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.orientation.Location = new System.Drawing.Point(0, 210);
            this.orientation.Name = "Selected orientation";
            this.orientation.Size = new System.Drawing.Size(600, 50);
            this.orientation.TabIndex = 0;
            this.orientation.Text = "orientation";
            this.orientation.TextAlign = System.Drawing.ContentAlignment.BottomCenter;
            // 
            // output label
            // 
            this.output.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom)
            | System.Windows.Forms.AnchorStyles.Left)
            | System.Windows.Forms.AnchorStyles.Right)));
            this.output.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.output.Font = new System.Drawing.Font("Microsoft Sans Serif", 15.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.output.Location = new System.Drawing.Point(0, 260);
            this.output.Name = "Send output";
            this.output.Size = new System.Drawing.Size(600, 50);
            this.output.TabIndex = 0;
            this.output.Text = "output";
            this.output.TextAlign = System.Drawing.ContentAlignment.BottomCenter;

            // 
            // info label
            // 
            this.info.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom)
            | System.Windows.Forms.AnchorStyles.Left)
            | System.Windows.Forms.AnchorStyles.Right)));
            this.info.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.info.Font = new System.Drawing.Font("Microsoft Sans Serif", 10.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.info.Location = new System.Drawing.Point(600, 200);
            this.info.Name = "info";
            this.info.Size = new System.Drawing.Size(400, 200);
            this.info.TabIndex = 0;
            this.info.Text = "1. Menu state carm\n" +
                "- Arm down and power --> base control\n" +
                "- Arm up and power --> carm control\n" +
                "2. Menu state axis\n" +
                "- Left right / up down --> flex / extend\n" +
                "- forward backward / anglular --> radial /ulnar\n" +
                "- rotation / orbiatl --> supination /pronation\n" +
                "3. Menu state motion\n" +
                "- hold fist and lift or lower arm for motion in each direction\n" +
                "\nGo back --> open hand";
            this.info.TextAlign = System.Drawing.ContentAlignment.TopLeft;
            //
            // button
            //
            this.button1.Font = new System.Drawing.Font("Microsoft Sans Serif", 11.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.button1.Location = new System.Drawing.Point(600, 0);
            this.button1.Name = "button1";
            this.button1.Size = new System.Drawing.Size(172, 41);
            this.button1.TabIndex = 7;
            this.button1.Text = "Start";
            this.button1.UseVisualStyleBackColor = true;
            this.button1.Click += new System.EventHandler(this.button1_Click);
        }

        private iM.ScopeMonitor smParadigmMenu;
        private System.Windows.Forms.Label state;
        private System.Windows.Forms.Label motion_activated;
        private System.Windows.Forms.Label orientation;
        private System.Windows.Forms.Label output;
        private System.Windows.Forms.Label info;
        private System.Windows.Forms.Label axis_info;
        private System.Windows.Forms.Button button1;
    }
}
